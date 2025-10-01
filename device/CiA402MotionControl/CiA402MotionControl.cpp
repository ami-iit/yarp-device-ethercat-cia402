// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "CiA402MotionControl.h"
#include "CiA402StateMachine.h"

#include <CiA402/EthercatManager.h>
#include <CiA402/LogComponent.h>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// STD
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace yarp::dev
{

/**
 * Private implementation holding configuration, runtime state, and helpers.
 * Concurrency:
 *  - variables.mutex guards feedback snapshots returned to YARP APIs
 *  - controlModeState.mutex guards target/active control modes
 *  - setPoints.mutex guards user setpoints and their first‑cycle latches
 * Units:
 *  - Positions: degrees (joint and motor)
 *  - Velocities: degrees/second
 *  - Torques: joint Nm (converted to motor Nm and then to per‑thousand for PDOs)
 */
struct CiA402MotionControl::Impl
{
    enum class PosSrc
    {
        S6064, // Use CiA 402 Position Actual Value (0x6064)
        Enc1, // Use encoder 1 position (0x2111:02 in PDO)
        Enc2 // Use encoder 2 position (0x2113:02 in PDO)
    };
    // Which object supplies velocity feedback for a given consumer (joint/motor)
    enum class VelSrc
    {
        S606C, // Use CiA 402 Velocity Actual Value (0x606C) [rpm]
        Enc1, // Use encoder 1 velocity (0x2111:03) [rpm]
        Enc2 // Use encoder 2 velocity (0x2113:03) [rpm]
    };
    // Where each encoder is physically mounted relative to the gearbox
    enum class Mount
    {
        None, // Not present / unknown
        Motor, // Pre‑gearbox (motor shaft)
        Joint // Post‑gearbox (load/joint shaft)
    };
    // Which sensor the drive uses internally for the closed loop
    enum class SensorSrc
    {
        Unknown, // Not reported via SDOs (fallback heuristics apply)
        Enc1, // Internal loop uses encoder 1
        Enc2 // Internal loop uses encoder 2
    };

    // Human-readable name for log messages
    static constexpr std::string_view kClassName = "CiA402MotionControl";

    // EtherCat manager
    ::CiA402::EthercatManager ethercatManager;

    // mutex to protect the run and the stop
    std::mutex CiA402MotionControlMutex;

    // Constants (unit conversions used throughout)
    static constexpr double MICROSECONDS_TO_SECONDS = 1e-6; // µs → s
    // Device timestamp wraps every ~42.949672 s (spec: (2^32 - 1)/100 µs)
    // Use +1 to account for the fractional remainder and avoid drift negative steps.
    // 42,949,673 µs
    static constexpr uint64_t TIMESTAMP_WRAP_PERIOD_US = (((uint64_t)1 << 32) - 1) / 100 + 1;
    // Heuristic for wrap detection: consider a wrap only if the value is jumps backward by more
    // than half of its range. Small backward steps can happen due to clock adjustments/resets and
    // must not be treated as wraps.
    static constexpr uint32_t TIMESTAMP_WRAP_HALF_RANGE = TIMESTAMP_WRAP_PERIOD_US / 2;
    // Parameters that come from the .xml file (see open())
    //--------------------------------------------------------------------------
    size_t numAxes{0}; // how many joints we expose to YARP
    int firstSlave{1}; // bus index of the first drive we care about
    std::string expectedName{}; // sanity-check each slave.name if not empty

    //--------------------------------------------------------------------------
    // Runtime bookkeeping
    //--------------------------------------------------------------------------
    std::vector<double> gearRatio; //  motor-revs : load-revs
    std::vector<double> gearRatioInv; // 1 / gearRatio
    std::vector<double> torqueConstants; // [Nm/A] from 0x2003:01
    std::vector<double> maxCurrentsA; // [A] from 0x6075
    std::vector<double> ratedMotorTorqueNm; // [Nm] from 0x6076
    std::vector<double> maxMotorTorqueNm; // [Nm] from 0x6072
    std::vector<uint32_t> enc1Res, enc2Res; // from 0x2110, 0x2112
    std::vector<double> enc1ResInv, enc2ResInv; // 1 / enc?Res
    std::vector<Mount> enc1Mount, enc2Mount; // from config (and SDO sanity)
    std::vector<PosSrc> posSrcJoint, posSrcMotor; // from config (0x2012:08 semantics)
    std::vector<VelSrc> velSrcJoint, velSrcMotor; // from config (0x2011:04 semantics)
    std::vector<SensorSrc> posLoopSrc; // drive internal pos loop source (0x2012:09)
    std::vector<SensorSrc> velLoopSrc; // drive internal vel loop source (0x2011:05)
    std::vector<double> velToDegS; // multiply device value to get deg/s
    std::vector<double> degSToVel; // multiply deg/s to get device value
    std::vector<bool> invertedMotionSenseDirection; // if true the torque, current, velocity and
                                                    // position have inverted sign
    struct Limits
    {
        std::vector<double> maxPositionLimitDeg; // [deg] from 0x607D:1
        std::vector<double> minPositionLimitDeg; // [deg] from 0x607D:2
        std::vector<bool> usePositionLimitsFromConfig; // if false, read 0x607D from SDO and cache
                                                       // here
        std::mutex mutex; // protects *all* the above vectors
    } limits;

    struct VariablesReadFromMotors
    {
        mutable std::mutex mutex; // protect the variables in this structure from concurrent access
        std::vector<double> motorEncoders; // for getMotorEncoders()
        std::vector<double> motorVelocities; // for getMotorVelocities()
        std::vector<double> motorAccelerations; // for getMotorAccelerations()
        std::vector<double> jointPositions; // for getJointPositions()
        std::vector<double> jointVelocities; // for getJointVelocities()
        std::vector<double> jointAccelerations; // for getJointAccelerations()
        std::vector<double> jointTorques; // for getJointTorques()
        std::vector<double> motorCurrents; // for getCurrents()
        std::vector<double> feedbackTime; // feedback time in seconds. Computed from
                                          // drive’s own internal timestep that starts when the
                                          // drive boots (or resets)
        std::vector<uint8_t> STO;
        std::vector<uint8_t> SBC;
        std::vector<std::string> jointNames; // for getAxisName()
        std::vector<bool> targetReached; // cached Statusword bit 10
        std::vector<double> driveTemperatures; // cached drive temperature if available

        void resizeContainers(std::size_t numAxes)
        {
            this->motorEncoders.resize(numAxes);
            this->motorVelocities.resize(numAxes);
            this->motorAccelerations.resize(numAxes);
            this->feedbackTime.resize(numAxes);
            this->jointPositions.resize(numAxes);
            this->jointVelocities.resize(numAxes);
            this->jointAccelerations.resize(numAxes);
            this->jointNames.resize(numAxes);
            this->jointTorques.resize(numAxes);
            this->motorCurrents.resize(numAxes);
            this->STO.resize(numAxes);
            this->SBC.resize(numAxes);
            this->targetReached.resize(numAxes);
            this->driveTemperatures.resize(numAxes);
        }
    };

    VariablesReadFromMotors variables;

    struct ControlModeState
    {
        std::vector<int> target; // what the user asked for
        std::vector<int> active; // what the drive is really doing
        std::vector<int> cstFlavor; // custom flavor for each axis (torque/current)
        std::vector<int> prevCstFlavor; // previous flavor, for change detection
        mutable std::mutex mutex; // protects *both* vectors

        void resize(std::size_t n)
        {
            target.assign(n, VOCAB_CM_IDLE);
            active = target;
            cstFlavor.assign(n, VOCAB_CM_UNKNOWN);
            prevCstFlavor = cstFlavor;
        }
    };
    ControlModeState controlModeState;

    struct SetPoints
    {
        std::mutex mutex; // protects the following vectors
        std::vector<double> jointTorques; // for setTorque()
        std::vector<double> jointVelocities; // for velocityMove() or joint velocity commands
        std::vector<double> motorCurrents; // for setCurrent()
        std::vector<bool> hasTorqueSP; // user provided a torque since entry?
        std::vector<bool> hasVelSP; // user provided a velocity since entry?
        std::vector<bool> hasCurrentSP; // user provided a current since entry?

        std::vector<double> jointPositionsDeg; // last requested target [deg] joint-side
        std::vector<int32_t> targetCounts; // computed target for 0x607A (loop-shaft)
        std::vector<bool> hasPosSP; // a new PP command available this cycle?

        std::vector<bool> isRelative; // this set-point is relative
        std::vector<bool> pulseHi; // drive CW bit4 high this cycle?
        std::vector<bool> pulseCoolDown; // bring bit4 low next

        void resize(std::size_t n)
        {
            jointTorques.resize(n);
            jointVelocities.resize(n);
            motorCurrents.resize(n);
            hasTorqueSP.assign(n, false);
            hasVelSP.assign(n, false);
            hasCurrentSP.assign(n, false);

            jointPositionsDeg.resize(n);
            targetCounts.resize(n);
            hasPosSP.assign(n, false);
            isRelative.assign(n, false);
            pulseHi.assign(n, false);
            pulseCoolDown.assign(n, false);
        }

        void reset()
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            std::fill(this->jointTorques.begin(), this->jointTorques.end(), 0.0);
            std::fill(this->jointVelocities.begin(), this->jointVelocities.end(), 0.0);
            std::fill(this->motorCurrents.begin(), this->motorCurrents.end(), 0.0);
            std::fill(this->hasTorqueSP.begin(), this->hasTorqueSP.end(), false);
            std::fill(this->hasCurrentSP.begin(), this->hasCurrentSP.end(), false);
            std::fill(this->hasVelSP.begin(), this->hasVelSP.end(), false);
            std::fill(this->jointPositionsDeg.begin(), this->jointPositionsDeg.end(), 0.0);
            std::fill(this->targetCounts.begin(), this->targetCounts.end(), 0);
            std::fill(this->hasPosSP.begin(), this->hasPosSP.end(), false);
            std::fill(this->isRelative.begin(), this->isRelative.end(), false);
            std::fill(this->pulseHi.begin(), this->pulseHi.end(), false);
            std::fill(this->pulseCoolDown.begin(), this->pulseCoolDown.end(), false);
        }

        void reset(int axis)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->jointTorques[axis] = 0.0;
            this->jointVelocities[axis] = 0.0;
            this->motorCurrents[axis] = 0.0;
            this->hasTorqueSP[axis] = false;
            this->hasCurrentSP[axis] = false;
            this->jointPositionsDeg[axis] = 0.0;
            this->targetCounts[axis] = 0;
            this->hasVelSP[axis] = false;
            this->hasPosSP[axis] = false;
            this->isRelative[axis] = false;
            this->pulseHi[axis] = false;
            this->pulseCoolDown[axis] = false;
        }
    };

    SetPoints setPoints;

    /* --------------- CiA-402 power-state machine ------------------------ */
    std::vector<std::unique_ptr<CiA402::StateMachine>> sm;

    // First-cycle latches and seed
    std::vector<bool> velLatched, trqLatched, currLatched;
    std::vector<bool> posLatched; // true if we have a valid target
    std::vector<double> torqueSeedNm;
    std::vector<int> lastActiveMode;

    // Timestamp unwrap (per-axis) for 32-bit drive timestamps
    std::vector<uint32_t> tsLastRaw; // last raw 32-bit timestamp read from drive
    std::vector<uint64_t> tsWraps; // number of wraps (increments on large backward jumps)

    struct PositionProfileState
    {
        std::mutex mutex;
        std::vector<double> ppRefSpeedDegS;
        std::vector<double> ppRefAccelerationDegSS;
        std::vector<bool> ppHaltRequested;
    };
    PositionProfileState ppState;

    bool opRequested{false}; // true after the first run() call

    // ------------- runtime profiling (run() duration and 5s average) -------------
    std::chrono::steady_clock::time_point avgWindowStart{std::chrono::steady_clock::now()};
    double runTimeAccumSec{0.0};
    std::size_t runTimeSamples{0};

    /** Dummy interaction mode for all the joints */
    yarp::dev::InteractionModeEnum dummyInteractionMode{
        yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF};

    Impl() = default;
    ~Impl() = default;

    // Convert joint degrees to loop-shaft counts for 0x607A, using the configured
    // position-loop source (Enc1/Enc2) and mount (Motor/Joint).
    int32_t jointDegToTargetCounts(size_t j, double jointDeg) const
    {
        // choose source and resolution
        uint32_t res = 0;
        Mount m = Mount::None;
        switch (posLoopSrc[j])
        {
        case SensorSrc::Enc1:
            res = enc1Res[j];
            m = enc1Mount[j];
            break;
        case SensorSrc::Enc2:
            res = enc2Res[j];
            m = enc2Mount[j];
            break;
        default:
            if (enc1Mount[j] != Mount::None)
            {
                res = enc1Res[j];
                m = enc1Mount[j];
            } else
            {
                res = enc2Res[j];
                m = enc2Mount[j];
            }
            break;
        }
        double shaftDeg = jointDeg;
        if (m == Mount::Motor)
            shaftDeg = jointDeg * gearRatio[j];
        else if (m == Mount::Joint)
            shaftDeg = jointDeg; // pass-through
        const double cnt = (res ? (shaftDeg / 360.0) * static_cast<double>(res) : 0.0);
        long long q = llround(cnt);
        if (q > std::numeric_limits<int32_t>::max())
            q = std::numeric_limits<int32_t>::max();
        if (q < std::numeric_limits<int32_t>::min())
            q = std::numeric_limits<int32_t>::min();
        return static_cast<int32_t>(q);
    }

    // Convert loop-shaft counts (0x607D/0x607A domain) back to joint degrees,
    // applying encoder mount, gear ratio, and motion-sense inversion.
    double targetCountsToJointDeg(size_t j, int32_t counts) const
    {
        // Apply motion-sense inversion: counts stored in the drive follow the loop shaft.
        // If our joint coordinate is inverted, flip the sign to get a consistent mapping.
        const bool inv = invertedMotionSenseDirection[j];
        const int32_t cAdj = inv ? -counts : counts;

        // choose source and resolution/mount just like jointDegToTargetCounts
        uint32_t res = 0;
        Mount m = Mount::None;
        switch (posLoopSrc[j])
        {
        case SensorSrc::Enc1:
            res = enc1Res[j];
            m = enc1Mount[j];
            break;
        case SensorSrc::Enc2:
            res = enc2Res[j];
            m = enc2Mount[j];
            break;
        default:
            if (enc1Mount[j] != Mount::None)
            {
                res = enc1Res[j];
                m = enc1Mount[j];
            } else
            {
                res = enc2Res[j];
                m = enc2Mount[j];
            }
            break;
        }

        // counts -> shaft degrees
        const double shaftDeg = res ? (double(cAdj) / double(res)) * 360.0 : 0.0;
        // shaft -> joint degrees depending on mount
        if (m == Mount::Motor)
        {
            return gearRatioInv[j] * shaftDeg; // joint = motor/gearRatio
        }
        // Joint-mounted or unknown → pass-through
        return shaftDeg;
    }

    bool setPositionCountsLimits(int axis, int32_t minCounts, int32_t maxCounts)
    {
        constexpr auto logPrefix = "[setPositionCountsLimits]";
        auto errorCode = this->ethercatManager.writeSDO<int32_t>(this->firstSlave + axis,
                                                                 0x607D,
                                                                 0x01,
                                                                 minCounts);
        if (errorCode != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "%s: setLimits: SDO write 0x607D:01 (min) failed for axis %d",
                    Impl::kClassName.data(),
                    axis);
            return false;
        }

        errorCode = this->ethercatManager.writeSDO<int32_t>(this->firstSlave + axis,
                                                            0x607D,
                                                            0x02,
                                                            maxCounts);
        if (errorCode != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "%s: setLimits: SDO write 0x607D:02 (max) failed for axis %d",
                    Impl::kClassName.data(),
                    axis);
            return false;
        }

        return true;
    }

    //--------------------------------------------------------------------------
    //  One-shot SDO reads – here we only fetch encoder resolutions
    //--------------------------------------------------------------------------
    bool readEncoderResolutions()
    {
        constexpr auto logPrefix = "[readEncoderResolutions]";

        enc1Res.resize(numAxes);
        enc2Res.resize(numAxes);
        enc1ResInv.resize(numAxes);
        enc2ResInv.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);

            uint32_t r1 = 0;
            auto e1 = ethercatManager.readSDO<uint32_t>(s, 0x2110, 0x03, r1);
            enc1Res[j] = (e1 == ::CiA402::EthercatManager::Error::NoError) ? r1 : 0;

            uint32_t r2 = 0;
            auto e2 = ethercatManager.readSDO<uint32_t>(s, 0x2112, 0x03, r2);
            enc2Res[j] = (e2 == ::CiA402::EthercatManager::Error::NoError) ? r2 : 0;
        }
        for (size_t j = 0; j < numAxes; ++j)
        {
            enc1ResInv[j] = enc1Res[j] ? 1.0 / double(enc1Res[j]) : 0.0;
            enc2ResInv[j] = enc2Res[j] ? 1.0 / double(enc2Res[j]) : 0.0;
        }

        // Print encoder resolution information for all axes
        yCInfo(CIA402, "%s successfully read encoder resolutions from SDO", logPrefix);
        for (size_t j = 0; j < numAxes; ++j)
        {
            yCDebug(CIA402,
                    "%s j=%zu enc1_resolution=%u (inv=%.9f), enc2_resolution=%u (inv=%.9f)",
                    logPrefix,
                    j,
                    enc1Res[j],
                    enc1ResInv[j],
                    enc2Res[j],
                    enc2ResInv[j]);
        }

        return true;
    }

    static std::tuple<double, double, const char*> decode60A9(uint32_t v)
    {
        // Known Synapticon encodings (prefix scales; middle bytes encode rev, last timebase)
        // RPM family
        if (v == 0x00B44700u)
            return {6.0, 1.0 / 6.0, "1 RPM"}; // 1 rpm -> 6 deg/s
        if (v == 0xFFB44700u)
            return {0.6, 1.0 / 0.6, "0.1 RPM"};
        if (v == 0xFEB44700u)
            return {0.06, 1.0 / 0.06, "0.01 RPM"};
        if (v == 0xFDB44700u)
            return {0.006, 1.0 / 0.006, "0.001 RPM"};
        // RPS family
        if (v == 0x00B40300u)
            return {360.0, 1.0 / 360.0, "1 RPS"};
        if (v == 0xFFB40300u)
            return {36.0, 1.0 / 36.0, "0.1 RPS"};
        if (v == 0xFEB40300u)
            return {3.6, 1.0 / 3.6, "0.01 RPS"};
        if (v == 0xFDB40300u)
            return {0.36, 1.0 / 0.36, "0.001 RPS"};

        // Fallback (spec says default is 1 RPM). Warn & assume 1 RPM.
        return {6.0, 1.0 / 6.0, "UNKNOWN (assume 1 RPM)"};
    }

    bool readSiVelocityUnits()
    {
        constexpr auto logPrefix = "[readSiVelocityUnits]";

        this->velToDegS.resize(numAxes);
        this->degSToVel.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            // default 1 RPM See
            // https://doc.synapticon.com/circulo/sw5.1/objects_html/6xxx/60a9.html?Highlight=0x60a9
            uint32_t raw = 0x00B44700u;
            auto e = ethercatManager.readSDO<uint32_t>(s, 0x60A9, 0x00, raw);

            auto [toDegS, toDev, name] = this->decode60A9(raw);
            velToDegS[j] = toDegS;
            degSToVel[j] = toDev;

            if (e == ::CiA402::EthercatManager::Error::NoError)
                yCDebug(CIA402,
                        "%s j=%zu 0x60A9=0x%08X velocity_unit=%s (1_unit=%.6f_deg/s)",
                        logPrefix,
                        j,
                        raw,
                        name,
                        toDegS);
            else
                yWarning("%s j=%zu failed to read 0x60A9, assuming 1_RPM (1_unit=6_deg/s)",
                         logPrefix,
                         j);
        }

        yCInfo(CIA402, "%s successfully read velocity conversion units from SDO", logPrefix);
        return true;
    }

    // Convert loop-shaft counts to joint degrees, using the configured
    double loopCountsToJointDeg(std::size_t j, double counts) const
    {
        // Pick loop sensor & mount (you already filled posLoopSrc[] in open())
        uint32_t res = 0;
        Mount mount = Mount::None;
        switch (posLoopSrc[j])
        {
        case SensorSrc::Enc1:
            res = enc1Res[j];
            mount = enc1Mount[j];
            break;
        case SensorSrc::Enc2:
            res = enc2Res[j];
            mount = enc2Mount[j];
            break;
        default: // fallback: prefer enc1 if available
            if (enc1Mount[j] != Mount::None && enc1Res[j] != 0)
            {
                res = enc1Res[j];
                mount = enc1Mount[j];
            } else
            {
                res = enc2Res[j];
                mount = enc2Mount[j];
            }
            break;
        }
        if (res == 0 || mount == Mount::None)
            return 0.0; // no info

        const double shaftDeg = (counts / double(res)) * 360.0; // degrees on the measured shaft
        // Convert to JOINT side depending on mount
        if (mount == Mount::Motor)
            return shaftDeg * gearRatioInv[j]; // motor → joint
        /* mount == Mount::Joint */ return shaftDeg; // already joint
    }

    bool setPositionWindowDeg(int j, double winDeg, double winTime_ms)
    {
        if (j < 0 || j >= static_cast<int>(this->numAxes))
        {
            yCError(CIA402,
                    "%s: setPositionWindowDeg: invalid joint index",
                    Impl::kClassName.data());
            return false;
        }
        const int s = this->firstSlave + j;

        uint32_t rawWin = 0;
        if (std::isinf(winDeg))
        {
            // Per doc: monitoring OFF → target reached bit stays 0.
            rawWin = 0xFFFFFFFFu;
        } else
        {
            rawWin = static_cast<uint32_t>(
                std::round(std::abs(this->jointDegToTargetCounts(std::size_t(j), winDeg))));
        }
        uint32_t rawTime = static_cast<uint32_t>(std::round(std::max(0.0, winTime_ms)));

        auto e1 = this->ethercatManager.writeSDO<uint32_t>(s, 0x6067, 0x00, rawWin);
        if (e1 != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "%s: setPositionWindowDeg: SDO 0x6067 write failed on joint %d",
                    Impl::kClassName.data(),
                    j);
            return false;
        }
        auto e2 = this->ethercatManager.writeSDO<uint32_t>(s, 0x6068, 0x00, rawTime);
        if (e2 != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "%s: setPositionWindowDeg: SDO 0x6068 write failed on joint %d",
                    Impl::kClassName.data(),
                    j);
            return false;
        }
        return true;
    }

    bool readMotorConstants()
    {
        constexpr auto logPrefix = "[readMotorConstants]";

        torqueConstants.resize(numAxes);
        maxCurrentsA.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            int32_t tcMicroNmA = 0;
            if (ethercatManager.readSDO<int32_t>(s, 0x2003, 0x02, tcMicroNmA)
                    != ::CiA402::EthercatManager::Error::NoError
                || tcMicroNmA == 0)
            {
                yWarning("%s j=%zu cannot read torque constant (0x2003:02) or zero, assuming 1.0 "
                         "Nm/A",
                         logPrefix,
                         j);
                torqueConstants[j] = 1.0;
            } else
            {
                // convert from µNm/A to Nm/A
                torqueConstants[j] = double(tcMicroNmA) * 1e-6;
            }

            // first of all we need to read the motor rated current
            // 0x6075:00  Motor Rated Current (UNSIGNED32)
            uint32_t ratedCurrentMilliA = 0;
            double ratedCurrentA = 0.0;
            if (ethercatManager.readSDO<uint32_t>(s, 0x6075, 0x00, ratedCurrentMilliA)
                    != ::CiA402::EthercatManager::Error::NoError
                || ratedCurrentMilliA == 0)
            {
                yWarning("%s j=%zu cannot read motor rated current (0x6075:00) or zero, assuming "
                         "1.0 A",
                         logPrefix,
                         j);
                ratedCurrentA = 1.0;
            } else
            {
                // convert from mA to A
                ratedCurrentA = double(ratedCurrentMilliA) * 1e-3;
            }

            // Now we can read the max current 0x6073:0	16 bit unsigned
            uint16_t maxPermille = 0;
            if (ethercatManager.readSDO<uint16_t>(s, 0x6073, 0x00, maxPermille)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yWarning("%s j=%zu cannot read max current (0x6073:00), assuming 1000 per_mille",
                         logPrefix,
                         j);
                maxPermille = 1000;
            }

            yCDebug(CIA402,
                    "%s j=%zu max_current=%u_permille (%.3fA), torque_constant=%.6f_Nm/A",
                    logPrefix,
                    j,
                    maxPermille,
                    (double(maxPermille) / 1000.0) * ratedCurrentA,
                    this->torqueConstants[j]);

            maxCurrentsA[j] = (double(maxPermille) / 1000.0) * ratedCurrentA;
        }

        yCInfo(CIA402, "%s successfully read motor constants from SDO", logPrefix);
        return true;
    }

    /**
     * Read the gear-ratio (motor-revs : load-revs) for every axis and cache both
     * the ratio and its inverse.
     *
     * 0x6091:01  numerator   (UNSIGNED32, default 1)
     * 0x6091:02  denominator (UNSIGNED32, default 1)
     *
     * If the object is missing or invalid the code silently falls back to 1:1.
     */
    bool readGearRatios()
    {
        constexpr auto logPrefix = "[readGearRatios]";

        gearRatio.resize(numAxes);
        gearRatioInv.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            uint32_t num = 1U; // default numerator
            uint32_t den = 1U; // default denominator

            // ---- numerator ------------------------------------------------------
            if (ethercatManager.readSDO<uint32_t>(slaveIdx, 0x6091, 0x01, num)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yWarning("%s j=%zu gear_ratio_numerator not available (0x6091:01), assuming 1",
                         logPrefix,
                         j);
                num = 1U;
            }

            // ---- denominator ----------------------------------------------------
            if (ethercatManager.readSDO<uint32_t>(slaveIdx, 0x6091, 0x02, den)
                    != ::CiA402::EthercatManager::Error::NoError
                || den == 0U)
            {
                yWarning("%s j=%zu gear_ratio_denominator not available/zero (0x6091:02), assuming "
                         "1",
                         logPrefix,
                         j);
                den = 1U;
            }

            yCDebug(CIA402,
                    "%s j=%zu gear_ratio=%u:%u (ratio=%.6f)",
                    logPrefix,
                    j,
                    num,
                    den,
                    static_cast<double>(num) / static_cast<double>(den));

            // ---- cache value ----------------------------------------------------
            gearRatio[j] = static_cast<double>(num) / static_cast<double>(den);
        }

        // Pre‑compute the inverse for fast use elsewhere -------------------------
        for (size_t j = 0; j < numAxes; ++j)
        {
            gearRatioInv[j] = (gearRatio[j] != 0.0) ? (1.0 / gearRatio[j]) : 0.0;
        }

        yCInfo(CIA402, "%s successfully read gear ratios from SDO", logPrefix);
        return true;
    }

    bool readTorqueValues()
    {
        constexpr auto logPrefix = "[readTorqueValues]";

        ratedMotorTorqueNm.resize(numAxes);
        maxMotorTorqueNm.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            uint32_t rated_mNm = 0;
            if (ethercatManager.readSDO<uint32_t>(s, 0x6076, 0x00, rated_mNm)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yCError(CIA402, "%s j=%zu cannot read rated torque (0x6076:00)", logPrefix, j);
                return false;
            }
            ratedMotorTorqueNm[j] = double(rated_mNm) / 1000.0; // motor Nm
        }
        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            uint16_t maxPerm = 0;
            if (ethercatManager.readSDO<uint16_t>(s, 0x6072, 0x00, maxPerm)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yCError(CIA402, "%s j=%zu cannot read max torque (0x6072:00)", logPrefix, j);
                return false;
            }
            maxMotorTorqueNm[j] = (double(maxPerm) / 1000.0) * ratedMotorTorqueNm[j];
            yCDebug(CIA402,
                    "%s j=%zu motor_rated_torque=%.3fNm max_torque=%.3fNm",
                    logPrefix,
                    j,
                    ratedMotorTorqueNm[j],
                    maxMotorTorqueNm[j]);
        }

        yCInfo(CIA402, "%s successfully read torque values from SDO", logPrefix);
        return true;
    }

    void setSDORefSpeed(int j, double spDegS)
    {
        // ----  map JOINT deg/s -> LOOP SHAFT deg/s (based on pos loop source + mount) ----
        double shaft_deg_s = spDegS; // default assume joint shaft
        switch (this->posLoopSrc[j])
        {
        case Impl::SensorSrc::Enc1:
            if (this->enc1Mount[j] == Impl::Mount::Motor)
                shaft_deg_s = spDegS * this->gearRatio[j];
            else if (this->enc1Mount[j] == Impl::Mount::Joint)
                shaft_deg_s = spDegS;
            break;
        case Impl::SensorSrc::Enc2:
            if (this->enc2Mount[j] == Impl::Mount::Motor)
                shaft_deg_s = spDegS * this->gearRatio[j];
            else if (this->enc2Mount[j] == Impl::Mount::Joint)
                shaft_deg_s = spDegS;
            break;
        case Impl::SensorSrc::Unknown:
        default:
            // Fallback: if we know which encoder is motor-mounted, assume that one; otherwise leave
            // as joint.
            if (this->enc1Mount[j] == Impl::Mount::Motor
                || this->enc2Mount[j] == Impl::Mount::Motor)
                shaft_deg_s = spDegS * this->gearRatio[j];
            break;
        }

        // Convert deg/s on the loop shaft -> device velocity units using 0x60A9
        const int s = this->firstSlave + j;
        const int32_t vel = static_cast<int32_t>(std::llround(shaft_deg_s * this->degSToVel[j]));
        (void)this->ethercatManager.writeSDO<int32_t>(s, 0x6081, 0x00, vel);
    }

    bool setSetPoints()
    {
        std::lock_guard<std::mutex> lock(this->setPoints.mutex);

        /**
         * Push user setpoints into RxPDOs according to the active control mode.
         *  - Torque (CST): joint Nm → motor Nm → per‑thousand of rated motor torque (0x6071)
         *  - Velocity (CSV): joint deg/s → loop‑shaft deg/s (mount aware) → rpm (0x60FF)
         * First‑cycle latches (velLatched/trqLatched) zero the command once when entering.
         */
        for (size_t j = 0; j < this->numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            auto rx = this->ethercatManager.getRxPDO(s);
            const int opMode = this->controlModeState.active[j];

            // clean rx targets
            rx->TargetPosition = 0;
            rx->TargetTorque = 0;
            rx->TargetVelocity = 0;

            // ---------- TORQUE (CST) ----------
            if (opMode == VOCAB_CM_TORQUE)
            {
                if (!this->trqLatched[j])
                {
                    rx->TargetTorque = 0;
                    this->trqLatched[j] = true;
                } else
                {
                    // YARP gives joint torque [Nm] → convert to MOTOR torque before 0x6071
                    const double jointNm = setPoints.hasTorqueSP[j] ? setPoints.jointTorques[j]
                                                                    : 0.0;
                    const double motorNm = (gearRatio[j] != 0.0) ? (jointNm / gearRatio[j]) : 0.0;

                    // 0x6071 is per-thousand of rated MOTOR torque (0x6076 in Nm)
                    const int16_t tq_thousand = static_cast<int16_t>(std::llround(
                        (ratedMotorTorqueNm[j] != 0.0 ? motorNm / ratedMotorTorqueNm[j] : 0.0)
                        * 1000.0));
                    rx->TargetTorque = this->invertedMotionSenseDirection[j] ? -tq_thousand
                                                                             : tq_thousand;
                }
            }

            // ---------- VELOCITY (CSV) ----------
            if (opMode == VOCAB_CM_VELOCITY)
            {
                if (!velLatched[j])
                {
                    rx->TargetVelocity = 0;
                    velLatched[j] = true;
                } else
                {
                    // YARP gives JOINT-side velocity [deg/s]
                    const double joint_deg_s = setPoints.hasVelSP[j] ? setPoints.jointVelocities[j]
                                                                     : 0.0;

                    // Command must be on the SHAFT used by the velocity loop:
                    //  - if loop sensor is motor-mounted → convert joint→motor (× gearRatio)
                    //  - if loop sensor is joint-mounted → pass through
                    double shaft_deg_s = joint_deg_s; // default: joint shaft

                    // The desired velocity setpoint depends on which encoder is used for
                    // the position loop and how it is mounted.
                    switch (posLoopSrc[j])
                    {
                    case Impl::SensorSrc::Enc1:
                        if (enc1Mount[j] == Impl::Mount::Motor)
                            shaft_deg_s = joint_deg_s * gearRatio[j];
                        else if (enc1Mount[j] == Impl::Mount::Joint)
                            shaft_deg_s = joint_deg_s;
                        // Impl::Mount::None → leave default (best-effort)
                        break;

                    case Impl::SensorSrc::Enc2:
                        if (enc2Mount[j] == Impl::Mount::Motor)
                            shaft_deg_s = joint_deg_s * gearRatio[j];
                        else if (enc2Mount[j] == Impl::Mount::Joint)
                            shaft_deg_s = joint_deg_s;
                        break;

                    case Impl::SensorSrc::Unknown:
                    default:
                        // Heuristic fallback: if we *know* the configured velocity feedback
                        // is motor-mounted, assume motor shaft; otherwise joint shaft.
                        // (Leave as joint if you don't keep velSrc vectors here.)
                        break;
                    }

                    // Convert deg/s → native velocity on the selected shaft for 0x60FF
                    const double vel = shaft_deg_s * this->degSToVel[j];
                    rx->TargetVelocity = static_cast<int32_t>(std::llround(vel))
                                         * (this->invertedMotionSenseDirection[j] ? -1 : 1);
                }
            }

            // ---------- POSITION (PP) ----------
            if (opMode == VOCAB_CM_POSITION)
            {

                if (this->ppState.ppHaltRequested[j])
                    rx->Controlword |= (1u << 8);
                else
                    rx->Controlword &= ~(1u << 8);
                // latch on first cycle in PP → do nothing until a user set-point arrives
                if (!posLatched[j])
                {
                    // keep bit 5 asserted in PP (single-set-point method)
                    rx->Controlword |= (1u << 5); // Change set immediately
                    rx->Controlword &= ~(1u << 4); // make sure New set-point is low first
                    posLatched[j] = true;
                    // seed target with current position until user commands a move
                    auto tx = this->ethercatManager.getTxView(s);
                    int32_t actualCounts = this->invertedMotionSenseDirection[j]
                                               ? -setPoints.targetCounts[j]
                                               : setPoints.targetCounts[j];
                    if (tx.has(CiA402::TxField::Position6064))
                    {
                        actualCounts = tx.get<int32_t>(CiA402::TxField::Position6064, actualCounts);
                    }

                    rx->TargetPosition = actualCounts;

                    const int32_t storedCounts
                        = this->invertedMotionSenseDirection[j] ? -actualCounts : actualCounts;
                    setPoints.targetCounts[j] = storedCounts;
                    setPoints.jointPositionsDeg[j] = this->targetCountsToJointDeg(j, actualCounts);
                    setPoints.isRelative[j] = false;

                    if (!setPoints.hasPosSP[j] && !setPoints.pulseHi[j])
                    {
                        setPoints.pulseHi[j] = true; // sync drive target to current position
                    }
                } else
                {
                    // (A) Always assert bit 5 in PP
                    rx->Controlword |= (1u << 5);

                    // (B) If the previous cycle drove bit4 high, bring it low now (one-shot pulse)
                    if (setPoints.pulseCoolDown[j])
                    {
                        rx->Controlword &= ~(1u << 4);
                        setPoints.pulseCoolDown[j] = false;
                    }

                    // (C) New set-point pending? write 0x607A and raise bit4
                    int32_t targetPositionCounts = 0;
                    if (setPoints.hasPosSP[j] || setPoints.pulseHi[j])
                    {
                        // Absolute/Relative selection (CW bit 6)
                        if (setPoints.isRelative[j])
                            rx->Controlword |= (1u << 6);
                        else
                            rx->Controlword &= ~(1u << 6);

                        // Target position (0x607A)
                        targetPositionCounts = this->invertedMotionSenseDirection[j]
                                                   ? -setPoints.targetCounts[j]
                                                   : setPoints.targetCounts[j];

                        // New set-point pulse (rising edge)
                        rx->Controlword |= (1u << 4);

                        // consume the request and arm cooldown to drop bit4 next cycle
                        setPoints.hasPosSP[j] = false;
                        setPoints.pulseHi[j] = false;
                        setPoints.pulseCoolDown[j] = true;
                    } else
                    {
                        auto tx = this->ethercatManager.getTxView(s);
                        targetPositionCounts = this->invertedMotionSenseDirection[j]
                                                   ? -setPoints.targetCounts[j]
                                                   : setPoints.targetCounts[j];
                        if (tx.has(CiA402::TxField::Position6064))
                        {
                            targetPositionCounts = tx.get<int32_t>(CiA402::TxField::Position6064,
                                                                   targetPositionCounts);
                        }

                        setPoints.targetCounts[j] = this->invertedMotionSenseDirection[j]
                                                        ? -targetPositionCounts
                                                        : targetPositionCounts;
                        setPoints.jointPositionsDeg[j]
                            = this->targetCountsToJointDeg(j, targetPositionCounts);
                        setPoints.isRelative[j] = false;
                    }

                    rx->TargetPosition = targetPositionCounts;
                }
            }

            if (opMode == VOCAB_CM_CURRENT)
            {
                if (!this->currLatched[j])
                {
                    // the current control is actually a torque control
                    rx->TargetTorque = 0;
                    this->currLatched[j] = true;
                } else
                {
                    // YARP gives Currents [A] → convert to MOTOR torque before 0x6071
                    const double currentA = setPoints.hasCurrentSP[j] ? setPoints.motorCurrents[j]
                                                                      : 0.0;

                    // convert the current in torque using the torque constant
                    const double torqueNm = currentA * this->torqueConstants[j];

                    // 0x6071 is per-thousand of rated MOTOR torque (0x6076 in Nm)
                    const int16_t tq_thousand = static_cast<int16_t>(std::llround(
                        (ratedMotorTorqueNm[j] != 0.0 ? torqueNm / ratedMotorTorqueNm[j] : 0.0)
                        * 1000.0));
                    rx->TargetTorque = this->invertedMotionSenseDirection[j] ? -tq_thousand
                                                                             : tq_thousand;
                }
            }
        }
        return true;
    }

    bool readFeedback()
    {
        std::lock_guard<std::mutex> lock(this->variables.mutex);

        // =====================================================================
        // SHAFT TRANSFORMATION HELPERS
        // =====================================================================
        // These lambdas handle the complex coordinate transformations between
        // motor shaft and joint/load shaft based on encoder mounting and gear ratios

        // Position transformation: converts degrees on a specific shaft to the requested side
        auto shaftFromMount_pos = [&](double deg, Mount m, size_t j, bool asMotor) -> double {
            // deg = input position in degrees on the physical shaft of mount 'm'
            // asMotor = true if we want result on motor shaft, false for joint shaft

            if (m == Impl::Mount::Motor)
            {
                // Input is on motor shaft
                return asMotor ? deg // Motor->Motor: no change
                               : deg * gearRatioInv[j]; // Motor->Joint: divide by gear ratio
            }
            if (m == Impl::Mount::Joint)
            {
                // Input is on joint/load shaft
                return asMotor ? deg * gearRatio[j] // Joint->Motor: multiply by gear ratio
                               : deg; // Joint->Joint: no change
            }
            return 0.0; // Impl::Mount::None or invalid
        };

        // Velocity transformation: same logic as position but for velocities
        auto shaftFromMount_vel = [&](double degs, Mount m, size_t j, bool asMotor) -> double {
            if (m == Impl::Mount::Motor)
            {
                return asMotor ? degs : degs * gearRatioInv[j];
            }
            if (m == Impl::Mount::Joint)
            {
                return asMotor ? degs * gearRatio[j] : degs;
            }
            return 0.0;
        };

        for (size_t j = 0; j < this->numAxes; ++j)
        {
            const int s = this->firstSlave + int(j);
            auto tx = this->ethercatManager.getTxView(s);

            // =====================================================================
            // Status word bits
            // =====================================================================
            const uint16_t sw = tx.get<uint16_t>(CiA402::TxField::Statusword);
            this->variables.targetReached[j] = (sw & (1u << 10)) != 0;

            // =====================================================================
            // RAW DATA EXTRACTION FROM PDOs
            // =====================================================================
            // Read raw encoder counts and velocities from the EtherCAT PDOs
            // These are the fundamental data sources before any interpretation

            // Position data (in encoder counts)
            const int32_t p6064 = tx.get<int32_t>(CiA402::TxField::Position6064, 0); // CiA402
                                                                                     // standard
                                                                                     // position
            const int32_t pE1 = tx.has(CiA402::TxField::Enc1Pos2111_02) // Encoder 1 position (if
                                                                        // mapped)
                                    ? tx.get<int32_t>(CiA402::TxField::Enc1Pos2111_02)
                                    : 0;
            const int32_t pE2 = tx.has(CiA402::TxField::Enc2Pos2113_02) // Encoder 2 position (if
                                                                        // mapped)
                                    ? tx.get<int32_t>(CiA402::TxField::Enc2Pos2113_02)
                                    : 0;

            // Velocity data (in RPM for CiA402, encoder-specific units for others)
            const int32_t v606C = tx.get<int32_t>(CiA402::TxField::Velocity606C, 0); // CiA402
                                                                                     // standard
                                                                                     // velocity
                                                                                     // (RPM)
            const int32_t vE1 = tx.has(CiA402::TxField::Enc1Vel2111_03) // Encoder 1 velocity (if
                                                                        // mapped)
                                    ? tx.get<int32_t>(CiA402::TxField::Enc1Vel2111_03)
                                    : 0;
            const int32_t vE2 = tx.has(CiA402::TxField::Enc2Vel2113_03) // Encoder 2 velocity (if
                                                                        // mapped)
                                    ? tx.get<int32_t>(CiA402::TxField::Enc2Vel2113_03)
                                    : 0;

            // =====================================================================
            // SOURCE INTERPRETATION HELPERS
            // =====================================================================
            // These lambdas convert raw data to degrees on the encoder's own shaft,
            // taking into account encoder resolution and the mounting location

            // Convert position source to degrees on its own physical shaft
            auto posDegOnOwnShaft = [&](PosSrc s) -> std::pair<double, Mount> {
                switch (s)
                {
                case Impl::PosSrc::Enc1:
                    // Direct encoder 1 readout: counts -> degrees using enc1 resolution
                    return {double(pE1) * enc1ResInv[j] * 360.0, enc1Mount[j]};

                case Impl::PosSrc::Enc2:
                    // Direct encoder 2 readout: counts -> degrees using enc2 resolution
                    return {double(pE2) * enc2ResInv[j] * 360.0, enc2Mount[j]};

                case Impl::PosSrc::S6064:
                default: {
                    // CiA402 standard object - interpretation depends on drive's loop source
                    // The drive tells us which encoder it uses internally via posLoopSrc
                    if (posLoopSrc[j] == Impl::SensorSrc::Enc1 && enc1ResInv[j] != 0.0)
                        return {double(p6064) * enc1ResInv[j] * 360.0, enc1Mount[j]};
                    if (posLoopSrc[j] == Impl::SensorSrc::Enc2 && enc2ResInv[j] != 0.0)
                        return {double(p6064) * enc2ResInv[j] * 360.0, enc2Mount[j]};

                    // Fallback: if loop source unknown, prefer enc1 if available
                    if (enc1Mount[j] != Impl::Mount::None && enc1ResInv[j] != 0.0)
                        return {double(p6064) * enc1ResInv[j] * 360.0, enc1Mount[j]};
                    if (enc2Mount[j] != Impl::Mount::None && enc2ResInv[j] != 0.0)
                        return {double(p6064) * enc2ResInv[j] * 360.0, enc2Mount[j]};
                    return {0.0, Impl::Mount::None};
                }
                }
            };
            // Convert velocity source to deg/s on its own physical shaft
            auto velDegSOnOwnShaft = [&](VelSrc s) -> std::pair<double, Mount> {
                const double k = this->velToDegS[j]; // 1 device unit -> k deg/s
                switch (s)
                {
                case Impl::VelSrc::Enc1:
                    // Direct encoder 1 velocity (already in RPM from PDO)
                    return {double(vE1) * k, enc1Mount[j]};

                case Impl::VelSrc::Enc2:
                    // Direct encoder 2 velocity (already in RPM from PDO)
                    return {double(vE2) * k, enc2Mount[j]};

                case Impl::VelSrc::S606C:
                default: {
                    // CiA402 standard velocity (0x606C)
                    // Per Synapticon docs, 0x606C is reported in the POSITION reference frame
                    // ("driving shaft"). That is, if a Position encoder is configured, 606C is
                    // scaled to that shaft (typically the joint/load). Only when no position
                    // encoder exists, it falls back to the velocity/torque encoder shaft (often
                    // motor).
                    // Therefore, select the mount based on the POSITION loop source (0x2012:09),
                    // not the velocity loop source.

                    // Primary: use reported position loop source
                    if (posLoopSrc[j] == Impl::SensorSrc::Enc1)
                        return {double(v606C) * k, enc1Mount[j]};
                    if (posLoopSrc[j] == Impl::SensorSrc::Enc2)
                        return {double(v606C) * k, enc2Mount[j]};

                    // Fallback heuristics when 0x2012:09 is Unknown:
                    // - If we have a joint-side encoder on enc2 (typical dual-encoder setup),
                    //   assume 606C is joint-side.
                    if (enc2Mount[j] == Impl::Mount::Joint)
                        return {double(v606C) * k, enc2Mount[j]};
                    // - Else prefer enc1 if present; otherwise enc2; else unknown
                    if (enc1Mount[j] != Impl::Mount::None)
                        return {double(v606C) * k, enc1Mount[j]};
                    if (enc2Mount[j] != Impl::Mount::None)
                        return {double(v606C) * k, enc2Mount[j]};
                    return {0.0, Impl::Mount::None};
                }
                }
            };

            // =====================================================================
            // POSITION FEEDBACK PROCESSING
            // =====================================================================
            // Apply the configured source selection and coordinate transformations
            {
                // Get raw position data from configured sources (on their own shafts)
                auto [degJ_src, mountJ] = posDegOnOwnShaft(posSrcJoint[j]); // Joint position source
                auto [degM_src, mountM] = posDegOnOwnShaft(posSrcMotor[j]); // Motor position source

                // Transform to the requested coordinate systems
                const double jointDeg = shaftFromMount_pos(degJ_src, mountJ, j, /*asMotor*/ false);
                const double motorDeg = shaftFromMount_pos(degM_src, mountM, j, /*asMotor*/ true);

                // Store in output variables
                this->variables.jointPositions[j]
                    = this->invertedMotionSenseDirection[j] ? -jointDeg : jointDeg;
                this->variables.motorEncoders[j] //
                    = this->invertedMotionSenseDirection[j] ? -motorDeg : motorDeg;
            }

            // =====================================================================
            // VELOCITY FEEDBACK PROCESSING
            // =====================================================================
            // Same logic as position but for velocities
            {
                // Get raw velocity data from configured sources (on their own shafts)
                auto [degsJ_src, mountJ] = velDegSOnOwnShaft(velSrcJoint[j]); // Joint velocity
                                                                              // source
                auto [degsM_src, mountM] = velDegSOnOwnShaft(velSrcMotor[j]); // Motor velocity
                                                                              // source

                // Transform to the requested coordinate systems
                const double jointDegS
                    = shaftFromMount_vel(degsJ_src, mountJ, j, /*asMotor*/ false);
                const double motorDegS = shaftFromMount_vel(degsM_src, mountM, j, /*asMotor*/ true);

                // Store in output variables
                this->variables.jointVelocities[j]
                    = this->invertedMotionSenseDirection[j] ? -jointDegS : jointDegS;
                this->variables.motorVelocities[j]
                    = this->invertedMotionSenseDirection[j] ? -motorDegS : motorDegS;
            }

            // =====================================================================
            // OTHER FEEDBACK PROCESSING
            // =====================================================================

            // Accelerations not provided by the drives (would require differentiation)
            this->variables.jointAccelerations[j] = 0.0;
            this->variables.motorAccelerations[j] = 0.0;

            // --------- Torque feedback (motor → joint conversion) ----------
            // CiA402 torque feedback (0x6077) is always motor-side, per-thousand of rated torque
            const double tq_per_thousand
                = double(tx.get<int16_t>(CiA402::TxField::Torque6077, 0)) / 1000.0;
            const double motorNm = tq_per_thousand * this->ratedMotorTorqueNm[j];
            // Convert motor torque to joint torque using gear ratio
            const double signedMotorNm = this->invertedMotionSenseDirection[j] ? -motorNm : motorNm;
            this->variables.jointTorques[j] = signedMotorNm * this->gearRatio[j];
            this->variables.motorCurrents[j] = signedMotorNm / this->torqueConstants[j];

            // --------- Safety signals (if mapped into PDOs) ----------
            // These provide real-time status of safety functions
            this->variables.STO[j] = tx.has(CiA402::TxField::STO_6621_01)
                                         ? tx.get<uint8_t>(CiA402::TxField::STO_6621_01)
                                         : 0; // Safe Torque Off status
            this->variables.SBC[j] = tx.has(CiA402::TxField::SBC_6621_02)
                                         ? tx.get<uint8_t>(CiA402::TxField::SBC_6621_02)
                                         : 0; // Safe Brake Control status

            // --------- Timestamp (if available) ----------
            // Provides drive-side timing information for synchronization
            if (tx.has(CiA402::TxField::Timestamp20F0))
            {
                const uint32_t raw = tx.get<uint32_t>(CiA402::TxField::Timestamp20F0, 0);
                // Unwrap 32-bit microsecond counter with threshold to avoid false wraps
                // due to small, non-monotonic clock adjustments.
                // Consider a wrap only if the backward jump is larger than half the range.
                if (raw < this->tsLastRaw[j])
                {
                    const uint32_t back = this->tsLastRaw[j] - raw;
                    if (back > TIMESTAMP_WRAP_HALF_RANGE)
                    {
                        this->tsWraps[j] += 1u;
                    }
                    // else: small backward step → no wraps increment
                }
                this->tsLastRaw[j] = raw;

                const uint64_t us_ext
                    = this->tsWraps[j] * TIMESTAMP_WRAP_PERIOD_US + static_cast<uint64_t>(raw);
                this->variables.feedbackTime[j]
                    = static_cast<double>(us_ext) * MICROSECONDS_TO_SECONDS;
            } else
            {
                this->variables.feedbackTime[j] = 0.0;
            }

            // the temperature is given in mC we need to convert Celsius
            this->variables.driveTemperatures[j]
                = tx.has(CiA402::TxField::TemperatureDrive)
                      ? double(tx.get<int32_t>(CiA402::TxField::TemperatureDrive, 0)) * 1e-3
                      : 0.0;
        }

        return true;
    }

    //--------------------------------------------------------------------------
    //  YARP ➜ CiA-402   (Op-mode sent in RxPDO::OpMode)
    //--------------------------------------------------------------------------
    static int yarpToCiaOp(int cm)
    {
        using namespace yarp::dev;
        switch (cm)
        {
        case VOCAB_CM_IDLE:
        case VOCAB_CM_FORCE_IDLE:
            return 0; // “No mode” → disables the power stage
        case VOCAB_CM_POSITION:
            return 1; // Profile-Position (PP)
        case VOCAB_CM_VELOCITY:
            return 9; // Cyclic-Synch-Velocity (CSV)
        case VOCAB_CM_TORQUE:
        case VOCAB_CM_CURRENT:
            return 10; // Cyclic-Synch-Torque  (CST)
        case VOCAB_CM_POSITION_DIRECT:
            return 8; // Cyclic-Synch-Position (CSP)
        default:
            return -1; // not supported
        }
    }

    //--------------------------------------------------------------------------
    //  CiA-402 ➜ YARP   (decode for diagnostics)
    //--------------------------------------------------------------------------
    int ciaOpToYarpWithFlavor(int op, int cstFlavor)
    {
        if (op == 10)
        {
            return (cstFlavor == VOCAB_CM_CURRENT) ? VOCAB_CM_CURRENT : VOCAB_CM_TORQUE;
        }
        return this->ciaOpToYarp(op);
    }

    static int ciaOpToYarp(int op)
    {
        using namespace yarp::dev;
        switch (op)
        {
        case 0:
            return VOCAB_CM_IDLE;
        case 1:
            return VOCAB_CM_POSITION;
        case 3:
            return VOCAB_CM_VELOCITY; // PP-vel still possible
        case 8:
            return VOCAB_CM_POSITION_DIRECT;
        case 9:
            return VOCAB_CM_VELOCITY;
        case 10:
            return VOCAB_CM_TORQUE;
        default:
            return VOCAB_CM_UNKNOWN;
        }
    }

    static std::string_view yarpToString(int op)
    {
        using namespace yarp::dev;
        switch (op)
        {
        case VOCAB_CM_IDLE:
            return "VOCAB_CM_IDLE";
        case VOCAB_CM_FORCE_IDLE:
            return "VOCAB_CM_FORCE_IDLE";
        case VOCAB_CM_POSITION:
            return "VOCAB_CM_POSITION";
        case VOCAB_CM_VELOCITY:
            return "VOCAB_CM_VELOCITY";
        case VOCAB_CM_TORQUE:
            return "VOCAB_CM_TORQUE";
        case VOCAB_CM_CURRENT:
            return "VOCAB_CM_CURRENT";
        case VOCAB_CM_POSITION_DIRECT:
            return "VOCAB_CM_POSITION_DIRECT";
        default:
            return "UNKNOWN";
        }
    }

    /**
     * Decode CiA 402 error code (0x603F:00) into a human‑readable string.
     * Includes explicit vendor codes seen in the field and broader family fallbacks.
     */
    std::string describe603F(uint16_t code)
    {
        switch (code)
        {
        case 0x0000:
            return "No fault";

        // === Your explicit mappings ===
        case 0x2220:
            return "Continuous over current (device internal)";
        case 0x2250:
            return "Short circuit (device internal)";
        case 0x2350:
            return "Load level fault (I2t, thermal state)";
        case 0x2351:
            return "Load level warning (I2t, thermal state)";
        case 0x3130:
            return "Phase failure";
        case 0x3131:
            return "Phase failure L1";
        case 0x3132:
            return "Phase failure L2";
        case 0x3133:
            return "Phase failure L3";
        case 0x3210:
            return "DC link over-voltage";
        case 0x3220:
            return "DC link under-voltage";
        case 0x3331:
            return "Field circuit interrupted";
        case 0x4210:
            return "Excess temperature device";
        case 0x4310:
            return "Excess temperature drive";
        case 0x5200:
            return "Control";
        case 0x5300:
            return "Operating unit";
        case 0x6010:
            return "Software reset (watchdog)";
        case 0x6320:
            return "Parameter error";
        case 0x7121:
            return "Motor blocked";
        case 0x7300:
            return "Sensor";
        case 0x7303:
            return "Resolver 1 fault";
        case 0x7304:
            return "Resolver 2 fault";
        case 0x7500:
            return "Communication";
        case 0x8612:
            return "Positioning controller (reference limit)";
        case 0xF002:
            return "Sub-synchronous run";

        default:
            // === Family fallbacks (upper byte) ===
            switch (code & 0xFF00)
            {
            case 0x2200:
                return "Current/device-internal fault";
            case 0x2300:
                return "Motor/output circuit fault";
            case 0x3100:
                return "Phase/mains supply issue";
            case 0x3200:
                return "DC link voltage issue";
            case 0x3300:
                return "Field/armature circuit issue";
            case 0x4200:
                return "Excess temperature (device)";
            case 0x4300:
                return "Excess temperature (drive)";
            case 0x5200:
                return "Control device hardware / limits";
            case 0x5300:
                return "Operating unit / safety";
            case 0x6000:
                return "Software reset / watchdog";
            case 0x6300:
                return "Parameter/configuration error";
            case 0x7100:
                return "Motor blocked / mechanical issue";
            case 0x7300:
                return "Sensor/encoder fault";
            case 0x7500:
                return "Communication/internal";
            case 0x8600:
                return "Positioning controller (vendor specific)";
            case 0xF000:
                return "Timing/performance warning";
            default: {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "Unknown 0x603F error: 0x%04X", code);
                return std::string(buf);
            }
            }
        }
    }

}; // struct Impl

//  CiA402MotionControl  —  ctor / dtor

CiA402MotionControl::CiA402MotionControl(double period, yarp::os::ShouldUseSystemClock useSysClock)
    : yarp::os::PeriodicThread(period, useSysClock, yarp::os::PeriodicThreadClock::Absolute)
    , m_impl(std::make_unique<Impl>())
{
}

CiA402MotionControl::CiA402MotionControl()
    : yarp::os::PeriodicThread(0.001 /*1 kHz*/,
                               yarp::os::ShouldUseSystemClock::Yes,
                               yarp::os::PeriodicThreadClock::Absolute)
    , m_impl(std::make_unique<Impl>())
{
}

CiA402MotionControl::~CiA402MotionControl() = default;

//  open()  —  bring the ring to OPERATIONAL and start the cyclic thread

bool CiA402MotionControl::open(yarp::os::Searchable& cfg)
{
    constexpr auto logPrefix = "[open]";

    // ---------------------------------------------------------------------
    // Parse driver parameters
    // ---------------------------------------------------------------------
    if (!cfg.check("ifname") || !cfg.find("ifname").isString())
    {
        yCError(CIA402, "%s: 'ifname' parameter is not a string", Impl::kClassName.data());
        return false;
    }
    if (!cfg.check("num_axes") || !cfg.find("num_axes").isInt32())
    {
        yCError(CIA402, "%s: 'num_axes' parameter is not an integer", Impl::kClassName.data());
        return false;
    }

    if (!cfg.check("period") || !cfg.find("period").isFloat64())
    {
        yCError(CIA402, "%s: 'period' parameter is not a float64", Impl::kClassName.data());
        return false;
    }
    const double period = cfg.find("period").asFloat64();
    if (period <= 0.0)
    {
        yCError(CIA402, "%s: 'period' parameter must be positive", Impl::kClassName.data());
        return false;
    }
    this->setPeriod(period);
    yCDebug(CIA402, "%s: using period = %.6f s", logPrefix, period);

    m_impl->numAxes = static_cast<size_t>(cfg.find("num_axes").asInt32());
    m_impl->firstSlave = cfg.check("first_slave", yarp::os::Value(1)).asInt32();
    if (cfg.check("expected_slave_name"))
        m_impl->expectedName = cfg.find("expected_slave_name").asString();

    // =========================================================================
    // ENCODER CONFIGURATION PARSING
    // =========================================================================
    // This system supports dual encoders per axis with flexible source selection:
    // - Enc1: Typically motor-mounted (high resolution, motor shaft feedback)
    // - Enc2: Typically joint-mounted (load-side feedback after gearbox)
    // - 6064/606C: Standard CiA402 objects that follow the configured loop sources
    //
    // Each feedback type (position/velocity for joint/motor) can independently
    // choose its source, allowing configurations like:
    // - Joint position from load encoder, motor position from motor encoder
    // - Joint velocity from load encoder, motor velocity from 606C

    // Parse helper functions for configuration strings
    auto parsePos = [&](const std::string& s) -> Impl::PosSrc {
        if (s == "enc1")
            return Impl::PosSrc::Enc1; // Use encoder 1 directly
        if (s == "enc2")
            return Impl::PosSrc::Enc2; // Use encoder 2 directly
        return Impl::PosSrc::S6064; // Use CiA402 standard object (follows loop source)
    };
    auto parseVel = [&](const std::string& s) -> Impl::VelSrc {
        if (s == "enc1")
            return Impl::VelSrc::Enc1; // Use encoder 1 velocity directly
        if (s == "enc2")
            return Impl::VelSrc::Enc2; // Use encoder 2 velocity directly
        return Impl::VelSrc::S606C; // Use CiA402 standard object (follows loop source)
    };
    auto parseMount = [&](const std::string& s) -> Impl::Mount {
        if (s == "motor")
            return Impl::Mount::Motor; // Encoder measures motor shaft (pre-gearbox)
        if (s == "joint")
            return Impl::Mount::Joint; // Encoder measures joint/load shaft (post-gearbox)
        if (s == "none")
            return Impl::Mount::None; // Encoder not present/not used
        yWarning("%s: invalid mount '%s' (allowed: none|motor|joint) → 'none'",
                 Impl::kClassName.data(),
                 s.c_str());
        return Impl::Mount::None;
    };

    auto extractListOfStringFromSearchable = [this](const yarp::os::Searchable& cfg,
                                                    const char* key,
                                                    const std::vector<std::string>& acceptedKeys,
                                                    std::vector<std::string>& result) -> bool {
        result.clear();

        if (!cfg.check(key))
        {
            yCError(CIA402, "%s: missing key '%s'", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Value& v = cfg.find(key);
        if (!v.isList())
        {
            yCError(CIA402, "%s: key '%s' is not a list", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Bottle* lst = v.asList();
        if (!lst)
        {
            yCError(CIA402,
                    "%s: internal error: list for key '%s' is null",
                    Impl::kClassName.data(),
                    key);
            return false;
        }

        const size_t expected = static_cast<size_t>(m_impl->numAxes);
        const size_t actual = static_cast<size_t>(lst->size());
        if (actual != expected)
        {
            yCError(CIA402,
                    "%s: list for key '%s' has incorrect size (%zu), expected (%zu)",
                    Impl::kClassName.data(),
                    key,
                    actual,
                    expected);
            return false;
        }

        result.reserve(expected);
        for (int i = 0; i < lst->size(); ++i)
        {
            const yarp::os::Value& elem = lst->get(i);

            if (!elem.isString())
            {
                yCError(CIA402,
                        "%s: element %d in list for key '%s' is not a string",
                        Impl::kClassName.data(),
                        i,
                        key);
                result.clear();
                return false;
            }

            const std::string val = elem.asString();

            // if acceptedKeys is non-empty, validate the value
            if (!acceptedKeys.empty())
            {
                if (std::find(acceptedKeys.begin(), acceptedKeys.end(), val) == acceptedKeys.end())
                {
                    yCError(CIA402,
                            "%s: invalid value '%s' in list for key '%s'",
                            Impl::kClassName.data(),
                            val.c_str(),
                            key);
                    result.clear();
                    return false;
                }
            }
            result.push_back(val);
        }

        return true;
    };

    auto extractListOfDoubleFromSearchable = [this](const yarp::os::Searchable& cfg,
                                                    const char* key,
                                                    std::vector<double>& result) -> bool {
        result.clear();

        if (!cfg.check(key))
        {
            yCError(CIA402, "%s: missing key '%s'", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Value& v = cfg.find(key);
        if (!v.isList())
        {
            yCError(CIA402, "%s: key '%s' is not a list", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Bottle* lst = v.asList();
        if (!lst)
        {
            yCError(CIA402,
                    "%s: internal error: list for key '%s' is null",
                    Impl::kClassName.data(),
                    key);
            return false;
        }

        const size_t expected = static_cast<size_t>(m_impl->numAxes);
        const size_t actual = static_cast<size_t>(lst->size());
        if (actual != expected)
        {
            yCError(CIA402,
                    "%s: list for key '%s' has incorrect size (%zu), expected (%zu)",
                    Impl::kClassName.data(),
                    key,
                    actual,
                    expected);
            return false;
        }

        result.reserve(expected);
        for (int i = 0; i < lst->size(); ++i)
        {
            const yarp::os::Value& elem = lst->get(i);

            if (!elem.isFloat64() && !elem.isInt32() && !elem.isInt64() && !elem.isFloat32())
            {
                yCError(CIA402,
                        "%s: element %d in list for key '%s' is not a number",
                        Impl::kClassName.data(),
                        i,
                        key);
                result.clear();
                return false;
            }

            result.push_back(elem.asFloat64());
        }

        return true;
    };

    auto extractListOfBoolFromSearchable = [this](const yarp::os::Searchable& cfg,
                                                  const char* key,
                                                  std::vector<bool>& result) -> bool {
        result.clear();

        if (!cfg.check(key))
        {
            yCError(CIA402, "%s: missing key '%s'", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Value& v = cfg.find(key);
        if (!v.isList())
        {
            yCError(CIA402, "%s: key '%s' is not a list", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Bottle* lst = v.asList();
        if (!lst)
        {
            yCError(CIA402,
                    "%s: internal error: list for key '%s' is null",
                    Impl::kClassName.data(),
                    key);
            return false;
        }

        const size_t expected = static_cast<size_t>(m_impl->numAxes);
        const size_t actual = static_cast<size_t>(lst->size());
        if (actual != expected)
        {
            yCError(CIA402,
                    "%s: list for key '%s' has incorrect size (%zu), expected (%zu)",
                    Impl::kClassName.data(),
                    key,
                    actual,
                    expected);
            return false;
        }

        result.reserve(expected);
        for (int i = 0; i < lst->size(); ++i)
        {
            const yarp::os::Value& elem = lst->get(i);

            if (!elem.isBool())
            {
                yCError(CIA402,
                        "%s: element %d in list for key '%s' is not a boolean",
                        Impl::kClassName.data(),
                        i,
                        key);
                result.clear();
                return false;
            }

            result.push_back(elem.asBool());
        }

        return true;
    };

    // Encoder mounting configuration (where each encoder is physically located)
    // enc1_mount must be list of "motor" or "joint"
    // enc2_mount must be list of "motor", "joint" or "none"
    std::vector<std::string> enc1MStr; // encoder 1 mount per axis
    std::vector<std::string> enc2MStr; // encoder 2 mount per axis

    if (!extractListOfStringFromSearchable(cfg, "enc1_mount", {"motor", "joint"}, enc1MStr))
        return false;
    if (!extractListOfStringFromSearchable(cfg, "enc2_mount", {"motor", "joint", "none"}, enc2MStr))
        return false;

    std::vector<std::string> posSrcJointStr;
    std::vector<std::string> posSrcMotorStr;
    std::vector<std::string> velSrcJointStr;
    std::vector<std::string> velSrcMotorStr;
    if (!extractListOfStringFromSearchable(cfg,
                                           "position_feedback_joint",
                                           {"6064", "enc1", "enc2"},
                                           posSrcJointStr))
        return false;
    if (!extractListOfStringFromSearchable(cfg,
                                           "position_feedback_motor",
                                           {"6064", "enc1", "enc2"},
                                           posSrcMotorStr))
        return false;
    if (!extractListOfStringFromSearchable(cfg,
                                           "velocity_feedback_joint",
                                           {"606C", "enc1", "enc2"},
                                           velSrcJointStr))
        return false;
    if (!extractListOfStringFromSearchable(cfg,
                                           "velocity_feedback_motor",
                                           {"606C", "enc1", "enc2"},
                                           velSrcMotorStr))
        return false;

    std::vector<double> positionWindowDeg; // position window for targetReached
    std::vector<double> timingWindowMs; // timing window for targetReached

    if (!extractListOfDoubleFromSearchable(cfg, "position_window_deg", positionWindowDeg))
        return false;
    if (!extractListOfDoubleFromSearchable(cfg, "timing_window_ms", timingWindowMs))
        return false;

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->enc1Mount.push_back(parseMount(enc1MStr[j]));
        m_impl->enc2Mount.push_back(parseMount(enc2MStr[j]));
        m_impl->posSrcJoint.push_back(parsePos(posSrcJointStr[j]));
        m_impl->posSrcMotor.push_back(parsePos(posSrcMotorStr[j]));
        m_impl->velSrcJoint.push_back(parseVel(velSrcJointStr[j]));
        m_impl->velSrcMotor.push_back(parseVel(velSrcMotorStr[j]));
    }

    const std::string ifname = cfg.find("ifname").asString();
    // Optional runtime knobs
    const int pdoTimeoutUs = cfg.check("pdo_timeout_us") ? cfg.find("pdo_timeout_us").asInt32()
                                                         : -1;
    const bool enableDc = cfg.check("enable_dc") ? cfg.find("enable_dc").asBool() : true;
    const int dcShiftNs = cfg.check("dc_shift_ns") ? cfg.find("dc_shift_ns").asInt32() : 0;
    yCInfo(CIA402, "%s opening EtherCAT manager on interface %s", logPrefix, ifname.c_str());

    if (!extractListOfBoolFromSearchable(cfg,
                                         "inverted_motion_sense_direction",
                                         m_impl->invertedMotionSenseDirection))
        return false;

    // ---------------------------------------------------------------------
    // Initialize the EtherCAT manager (ring in SAFE‑OP)
    // ---------------------------------------------------------------------

    const auto ecErr = m_impl->ethercatManager.init(ifname);
    if (ecErr != ::CiA402::EthercatManager::Error::NoError)
    {
        yCError(CIA402,
                "%s EtherCAT initialization failed with error code %d",
                logPrefix,
                static_cast<int>(ecErr));
        return false;
    }

    if (pdoTimeoutUs > 0)
    {
        m_impl->ethercatManager.setPdoTimeoutUs(pdoTimeoutUs);
        yCInfo(CIA402,
               "%s PDO receive timeout set to %d us",
               logPrefix,
               m_impl->ethercatManager.getPdoTimeoutUs());
    } else
    {
        yCInfo(CIA402,
               "%s using default PDO receive timeout of %d us",
               logPrefix,
               m_impl->ethercatManager.getPdoTimeoutUs());
    }

    // We'll enable DC later, after going OP, using the configured period
    const uint32_t cycleNs = static_cast<uint32_t>(std::llround(this->getPeriod() * 1e9));

    // =========================================================================
    // DRIVE LOOP SOURCE CONFIGURATION READING
    // =========================================================================
    // Read which encoders the drive uses internally for position/velocity loops
    // This affects how 6064/606C values should be interpreted

    // Loop-source readers (local lambdas)
    auto readPosLoopSrc = [&](size_t j) -> Impl::SensorSrc {
        const int s = m_impl->firstSlave + int(j);
        uint8_t src = 0;
        auto e = m_impl->ethercatManager.readSDO<uint8_t>(s, 0x2012, 0x09, src);
        if (e != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402,
                    "%s: failed to read position loop source (joint %zu)",
                    Impl::kClassName.data(),
                    j);
            return Impl::SensorSrc::Unknown;
        }
        return (src == 1)   ? Impl::SensorSrc::Enc1
               : (src == 2) ? Impl::SensorSrc::Enc2
                            : Impl::SensorSrc::Unknown;
    };
    auto readVelLoopSrc = [&](size_t j) -> Impl::SensorSrc {
        const int s = m_impl->firstSlave + int(j);
        uint8_t src = 0;
        auto e = m_impl->ethercatManager.readSDO<uint8_t>(s, 0x2011, 0x05, src);
        if (e != ::CiA402::EthercatManager::Error::NoError)
            return Impl::SensorSrc::Unknown;
        return (src == 1)   ? Impl::SensorSrc::Enc1
               : (src == 2) ? Impl::SensorSrc::Enc2
                            : Impl::SensorSrc::Unknown;
    };

    // --------- PDO Mapping Availability Checks ----------
    // These check if encoder data was successfully mapped into the PDOs
    // during the EtherCAT initialization phase
    auto hasEnc1Pos = [&](size_t j) {
        const int s = m_impl->firstSlave + int(j);
        return m_impl->ethercatManager.getTxView(s).has(CiA402::TxField::Enc1Pos2111_02);
    };
    auto hasEnc1Vel = [&](size_t j) {
        const int s = m_impl->firstSlave + int(j);
        return m_impl->ethercatManager.getTxView(s).has(CiA402::TxField::Enc1Vel2111_03);
    };
    auto hasEnc2Pos = [&](size_t j) {
        const int s = m_impl->firstSlave + int(j);
        return m_impl->ethercatManager.getTxView(s).has(CiA402::TxField::Enc2Pos2113_02);
    };
    auto hasEnc2Vel = [&](size_t j) {
        const int s = m_impl->firstSlave + int(j);
        return m_impl->ethercatManager.getTxView(s).has(CiA402::TxField::Enc2Vel2113_03);
    };

    yCInfo(CIA402,
           "%s using %zu axes, EtherCAT slaves %d to %d",
           logPrefix,
           m_impl->numAxes,
           m_impl->firstSlave,
           m_impl->firstSlave + int(m_impl->numAxes) - 1);

    // --------- Read drive-internal loop sources ----------
    // These determine how CiA402 standard objects (6064/606C) should be interpreted
    m_impl->posLoopSrc.resize(m_impl->numAxes, Impl::SensorSrc::Unknown);
    m_impl->velLoopSrc.resize(m_impl->numAxes, Impl::SensorSrc::Unknown);
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->posLoopSrc[j] = readPosLoopSrc(j);
        m_impl->velLoopSrc[j] = readVelLoopSrc(j);
    }

    // =========================================================================
    // CONFIGURATION VALIDATION
    // =========================================================================
    // Strict validation: requested encoders must be both mapped in PDOs AND
    // have valid mount points. This prevents runtime errors from misconfiguration.
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        auto bad = [&](const char* what) {
            yCError(CIA402, "%s j=%zu invalid configuration: %s", logPrefix, j, what);
            return true;
        };

        // --------- Position feedback validation ----------
        if (m_impl->posSrcJoint[j] == Impl::PosSrc::Enc1
            && (m_impl->enc1Mount[j] == Impl::Mount::None || !hasEnc1Pos(j)))
        {
            if (bad("pos_joint=enc1 but enc1 not mounted/mapped"))
                return false;
        }
        if (m_impl->posSrcJoint[j] == Impl::PosSrc::Enc2
            && (m_impl->enc2Mount[j] == Impl::Mount::None || !hasEnc2Pos(j)))
        {
            if (bad("pos_joint=enc2 but enc2 not mounted/mapped"))
                return false;
        }

        if (m_impl->posSrcMotor[j] == Impl::PosSrc::Enc1
            && (m_impl->enc1Mount[j] == Impl::Mount::None || !hasEnc1Pos(j)))
        {
            if (bad("pos_motor=enc1 but enc1 not mounted/mapped"))
                return false;
        }
        if (m_impl->posSrcMotor[j] == Impl::PosSrc::Enc2
            && (m_impl->enc2Mount[j] == Impl::Mount::None || !hasEnc2Pos(j)))
        {
            if (bad("pos_motor=enc2 but enc2 not mounted/mapped"))
                return false;
        }

        // --------- Velocity feedback validation ----------
        if (m_impl->velSrcJoint[j] == Impl::VelSrc::Enc1
            && (m_impl->enc1Mount[j] == Impl::Mount::None || !hasEnc1Vel(j)))
        {
            if (bad("vel_joint=enc1 but enc1 not mounted/mapped"))
                return false;
        }
        if (m_impl->velSrcJoint[j] == Impl::VelSrc::Enc2
            && (m_impl->enc2Mount[j] == Impl::Mount::None || !hasEnc2Vel(j)))
        {
            if (bad("vel_joint=enc2 but enc2 not mounted/mapped"))
                return false;
        }

        if (m_impl->velSrcMotor[j] == Impl::VelSrc::Enc1
            && (m_impl->enc1Mount[j] == Impl::Mount::None || !hasEnc1Vel(j)))
        {
            if (bad("vel_motor=enc1 but enc1 not mounted/mapped"))
                return false;
        }
        if (m_impl->velSrcMotor[j] == Impl::VelSrc::Enc2
            && (m_impl->enc2Mount[j] == Impl::Mount::None || !hasEnc2Vel(j)))
        {
            if (bad("vel_motor=enc2 but enc2 not mounted/mapped"))
                return false;
        }
    }

    // ---------------------------------------------------------------------
    // Read static SDO parameters (encoders, velocity units, gear ratios, motor limits)
    // ---------------------------------------------------------------------
    if (!m_impl->readEncoderResolutions())
    {
        yCError(CIA402, "%s failed to read encoder resolutions from SDO", logPrefix);
        return false;
    }
    if (!m_impl->readSiVelocityUnits())
    {
        yCError(CIA402, "%s failed to read velocity conversions from SDO", logPrefix);
        return false;
    }
    if (!m_impl->readGearRatios())
    {
        yCError(CIA402, "%s failed to read gear ratios from SDO", logPrefix);
        return false;
    }
    if (!m_impl->readMotorConstants())
    {
        yCError(CIA402, "%s failed to read motor constants from SDO", logPrefix);
        return false;
    }
    if (!m_impl->readTorqueValues())
    {
        yCError(CIA402, "%s failed to read torque values from SDO", logPrefix);
        return false;
    }

    // ---------------------------------------------------------------------
    // Set the position limits (SDO 607D)
    // ---------------------------------------------------------------------
    if (!extractListOfDoubleFromSearchable(cfg,
                                           "pos_limit_min_deg",
                                           m_impl->limits.minPositionLimitDeg))
    {
        yCError(CIA402, "%s failed to parse pos_limit_min_deg", logPrefix);
        return false;
    }
    if (!extractListOfDoubleFromSearchable(cfg,
                                           "pos_limit_max_deg",
                                           m_impl->limits.maxPositionLimitDeg))
    {
        yCError(CIA402, "%s failed to parse pos_limit_max_deg", logPrefix);
        return false;
    }
    if (!extractListOfBoolFromSearchable(cfg,
                                         "use_position_limits_from_config",
                                         m_impl->limits.usePositionLimitsFromConfig))
    {
        yCError(CIA402, "%s failed to parse use_position_limits_from_config", logPrefix);
        return false;
    }

    if (m_impl->limits.minPositionLimitDeg.size() != m_impl->numAxes
        || m_impl->limits.maxPositionLimitDeg.size() != m_impl->numAxes
        || m_impl->limits.usePositionLimitsFromConfig.size() != m_impl->numAxes)
    {
        yCError(CIA402,
                "%s position limit lists must have exactly %zu elements",
                logPrefix,
                m_impl->numAxes);
        return false;
    }

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        const bool inv = m_impl->invertedMotionSenseDirection[j];

        if (m_impl->limits.usePositionLimitsFromConfig[j])
        {
            // Convert joint degrees to loop-shaft counts and write to SDO 0x607D
            const int32_t lowerCounts
                = m_impl->jointDegToTargetCounts(j, m_impl->limits.minPositionLimitDeg[j]);
            const int32_t upperCounts
                = m_impl->jointDegToTargetCounts(j, m_impl->limits.maxPositionLimitDeg[j]);

            int32_t minCounts = 0;
            int32_t maxCounts = 0;
            if (!inv)
            {
                minCounts = lowerCounts;
                maxCounts = upperCounts;
            } else
            {
                // Decreasing mapping f(x) = -counts(x) → [L, U] maps to [f(U), f(L)]
                minCounts = -upperCounts;
                maxCounts = -lowerCounts;
            }

            if (!m_impl->setPositionCountsLimits(j, minCounts, maxCounts))
            {
                yCError(CIA402, "%s j=%zu failed to set position limits", logPrefix, j);
                return false;
            }
        } else
        {
            // Read 0x607D:min/max from SDO, convert to degrees, and cache into limits vectors.
            const int s = m_impl->firstSlave + static_cast<int>(j);
            int32_t minCounts = 0;
            int32_t maxCounts = 0;
            auto e1 = m_impl->ethercatManager.readSDO<int32_t>(s, 0x607D, 0x01, minCounts);
            auto e2 = m_impl->ethercatManager.readSDO<int32_t>(s, 0x607D, 0x02, maxCounts);
            if (e1 != ::CiA402::EthercatManager::Error::NoError
                || e2 != ::CiA402::EthercatManager::Error::NoError)
            {
                yCError(CIA402, "%s j=%zu failed to read position limits from SDO", logPrefix, j);
                return false;
            }

            // Convert counts back to joint-space degrees.
            double minDeg = 0.0;
            double maxDeg = 0.0;
            if (!inv)
            {
                minDeg = m_impl->targetCountsToJointDeg(j, minCounts);
                maxDeg = m_impl->targetCountsToJointDeg(j, maxCounts);
            } else
            {
                // When inverted, SDO holds [-U, -L]; swap to recover [L, U] in degrees.
                minDeg = m_impl->targetCountsToJointDeg(j, maxCounts);
                maxDeg = m_impl->targetCountsToJointDeg(j, minCounts);
            }

            m_impl->limits.minPositionLimitDeg[j] = minDeg;
            m_impl->limits.maxPositionLimitDeg[j] = maxDeg;
        }
    }

    // ---------------------------------------------------------------------
    // Idle all drives (switch‑on disabled, no mode selected)
    // ---------------------------------------------------------------------
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        const int slave = m_impl->firstSlave + static_cast<int>(j);
        auto* rx = m_impl->ethercatManager.getRxPDO(slave);
        if (!rx)
        {
            yCError(CIA402, "%s invalid slave index %d for axis %zu", logPrefix, slave, j);
            return false;
        }
        rx->Controlword = 0x0000;
        rx->OpMode = 0;
    }

    // Send one frame so the outputs take effect
    if (m_impl->ethercatManager.sendReceive() != ::CiA402::EthercatManager::Error::NoError)
    {
        yCError(CIA402, "%s initial EtherCAT send/receive after SDO reading failed", logPrefix);
        return false;
    }

    // ---------------------------------------------------------------------
    // Configure position windows (targetReached thresholds)
    // ---------------------------------------------------------------------
    for (int j = 0; j < m_impl->numAxes; ++j)
    {
        if (!m_impl->setPositionWindowDeg(j, positionWindowDeg[j], timingWindowMs[j]))
        {
            yCError(CIA402, "%s j=%d failed to set position window", logPrefix, j);
            return false;
        }
    }

    // ---------------------------------------------------------------------
    // Allocate runtime containers and CiA‑402 state machines
    // ---------------------------------------------------------------------

    m_impl->variables.resizeContainers(m_impl->numAxes);
    m_impl->setPoints.resize(m_impl->numAxes);
    m_impl->setPoints.reset();
    m_impl->controlModeState.resize(m_impl->numAxes);
    m_impl->sm.resize(m_impl->numAxes);
    m_impl->velLatched.assign(m_impl->numAxes, false);
    m_impl->trqLatched.assign(m_impl->numAxes, false);
    m_impl->currLatched.assign(m_impl->numAxes, false);
    m_impl->posLatched.assign(m_impl->numAxes, false);
    m_impl->torqueSeedNm.assign(m_impl->numAxes, 0.0);
    m_impl->tsLastRaw.assign(m_impl->numAxes, 0u);
    m_impl->tsWraps.assign(m_impl->numAxes, 0u);
    m_impl->ppState.ppRefSpeedDegS.assign(m_impl->numAxes, 0.0);
    m_impl->ppState.ppRefAccelerationDegSS.assign(m_impl->numAxes, 0.0);
    m_impl->ppState.ppHaltRequested.assign(m_impl->numAxes, false);

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->sm[j] = std::make_unique<CiA402::StateMachine>();
        m_impl->sm[j]->reset();
    }

    // get the axes names (if available)
    if (!extractListOfStringFromSearchable(cfg, "axes_names", {}, m_impl->variables.jointNames))
        return false;

    constexpr double initialPositionVelocityDegs = 10;
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        // Cache for getRefSpeed()
        {
            std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
            m_impl->ppState.ppRefSpeedDegS[j] = initialPositionVelocityDegs;
        }
        m_impl->setSDORefSpeed(j, initialPositionVelocityDegs);
    }

    yCInfo(CIA402, "%s opened %zu axes, initialization complete", logPrefix, m_impl->numAxes);

    // ---------------------------------------------------------------------
    // Transition to OPERATIONAL and enable DC
    // ---------------------------------------------------------------------
    {
        const auto opErr = m_impl->ethercatManager.goOperational();
        if (opErr != ::CiA402::EthercatManager::Error::NoError)
        {
            yCError(CIA402, "%s failed to enter OPERATIONAL state", logPrefix);
            return false;
        }

        if (enableDc)
        {
            const auto dcErr
                = m_impl->ethercatManager.enableDCSync0(cycleNs, /*shift_ns=*/dcShiftNs);
            if (dcErr != ::CiA402::EthercatManager::Error::NoError)
            {
                yCError(CIA402, "%s failed to enable DC SYNC0", logPrefix);
                return false;
            }
        } else
        {
            yWarning("%s DC synchronization disabled by configuration", logPrefix);
        }
    }

    // ---------------------------------------------------------------------
    // Launch the device thread
    // ---------------------------------------------------------------------
    if (!this->start())
    {
        yCError(CIA402, "%s failed to start device thread", logPrefix);
        return false;
    }

    return true;
}

//  close()  —  stop the thread & release the NIC
bool CiA402MotionControl::close()
{
    std::lock_guard<std::mutex> closeGuard(m_impl->CiA402MotionControlMutex);
    this->suspend();
    this->stop(); // PeriodicThread → graceful stop
    yCInfo(CIA402, "%s: EtheCAT master closed.", Impl::kClassName.data());
    return true;
}

//  run()  —  gets called every period (real-time control loop)
/**
 * Control loop phases:
 *  1) Apply user‑requested control modes (CiA‑402 power state machine)
 *  2) Push user setpoints to PDOs (units and shaft conversions handled)
 *  3) Cyclic exchange over EtherCAT (outputs sent / inputs read)
 *  4) Read back feedback (encoders, torque, safety, timestamp)
 *  5) Diagnostics and bookkeeping (active mode tracking, latching, inhibit)
 */
void CiA402MotionControl::run()
{
    constexpr auto logPrefix = "[run]";
    const auto tStart = std::chrono::steady_clock::now();

    // lock the mutex
    std::lock_guard<std::mutex> runGuard(m_impl->CiA402MotionControlMutex);

    /* ------------------------------------------------------------------
     * 1.  APPLY USER-REQUESTED CONTROL MODES (CiA-402 power machine)
     * ----------------------------------------------------------------*/
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);

        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            const int slaveIdx = m_impl->firstSlave + static_cast<int>(j);
            ::CiA402::RxPDO* rx = m_impl->ethercatManager.getRxPDO(slaveIdx);
            auto tx = m_impl->ethercatManager.getTxView(slaveIdx);

            const int tgt = m_impl->controlModeState.target[j];

            // If in FAULT  the only way to escape from the fault is to do FORCE IDLE
            if (m_impl->controlModeState.active[j] == VOCAB_CM_HW_FAULT
                && tgt == VOCAB_CM_FORCE_IDLE)
            {
                const auto cmd = m_impl->sm[j]->faultReset(); // CW=0x0080
                rx->Controlword = cmd.controlword;
                rx->OpMode = 0; // neutral
                // Clear user setpoints/latches immediately
                m_impl->setPoints.reset((int)j);
                m_impl->velLatched[j] = m_impl->trqLatched[j] = m_impl->currLatched[j] = false;
                continue; // skip normal update-path this cycle
            }

            /* ------------ normal control-mode path --------------------- */
            const int8_t opReq = Impl::yarpToCiaOp(tgt); // −1 = unsupported
            if (opReq < 0)
            { // ignore unknown modes
                continue;
            }

            const bool hwInhibit = (tx.has(CiA402::TxField::STO_6621_01)
                                    && tx.get<uint8_t>(CiA402::TxField::STO_6621_01))
                                   || (tx.has(CiA402::TxField::SBC_6621_02)
                                       && tx.get<uint8_t>(CiA402::TxField::SBC_6621_02));
            const uint16_t sw = tx.get<uint16_t>(CiA402::TxField::Statusword, 0);
            const int8_t od = tx.get<int8_t>(CiA402::TxField::OpModeDisplay, 0);
            CiA402::StateMachine::Command cmd = m_impl->sm[j]->update(sw, od, opReq, hwInhibit);

            rx->Controlword = cmd.controlword;
            if (cmd.writeOpMode)
            {
                rx->OpMode = cmd.opMode;
            }
        }
    } /* mutex unlocked – PDOs are now ready to send */

    /* ------------------------------------------------------------------
     * 2.  SET USER-REQUESTED SETPOINTS (torque, position, velocity)
     * ----------------------------------------------------------------*/
    m_impl->setSetPoints();

    /* ------------------------------------------------------------------
     * 3.  CYCLIC EXCHANGE  (send outputs / read inputs)
     * ----------------------------------------------------------------*/
    if (m_impl->ethercatManager.sendReceive() != ::CiA402::EthercatManager::Error::NoError)
    {
        yCError(CIA402,
                "%s sendReceive() failed, expected_wkc=%d got_wkc=%d",
                logPrefix,
                m_impl->ethercatManager.getExpectedWorkingCounter(),
                m_impl->ethercatManager.getWorkingCounter());
        m_impl->ethercatManager.dumpDiagnostics();
    }

    /* ------------------------------------------------------------------
     * 4.  COPY ENCODERS & OTHER FEEDBACK
     * ----------------------------------------------------------------*/
    m_impl->readFeedback();

    /* ------------------------------------------------------------------
     * 5.  DIAGNOSTICS  (fill active[] according to feedback)
     * ----------------------------------------------------------------*/
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);

        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            const int slaveIdx = m_impl->firstSlave + static_cast<int>(j);
            auto tx = m_impl->ethercatManager.getTxView(slaveIdx);
            bool flavourChanged = false;

            const bool hwInhibit = (tx.has(CiA402::TxField::STO_6621_01)
                                    && tx.get<uint8_t>(CiA402::TxField::STO_6621_01))
                                   || (tx.has(CiA402::TxField::SBC_6621_02)
                                       && tx.get<uint8_t>(CiA402::TxField::SBC_6621_02));
            const bool enabled = CiA402::StateMachine::isOperationEnabled(
                tx.get<uint16_t>(CiA402::TxField::Statusword, 0));
            const bool inFault
                = CiA402::StateMachine::isFault(tx.get<uint16_t>(CiA402::TxField::Statusword, 0))
                  || CiA402::StateMachine::isFaultReactionActive(
                      tx.get<uint16_t>(CiA402::TxField::Statusword, 0));

            int newActive = VOCAB_CM_IDLE;
            if (inFault)
            {
                newActive = VOCAB_CM_HW_FAULT;
            } else if (!hwInhibit && enabled)
            {
                const int activeOp = m_impl->sm[j]->getActiveOpMode();
                // If the drive is in CST (10), reflect the user-facing label (CURRENT or TORQUE)
                // that was last requested for this axis. Otherwise use the generic mapping.
                if (activeOp == 10)
                {
                    // Guarded by the same mutex we already hold
                    int flavor = m_impl->controlModeState.cstFlavor[j];
                    // Safety net: constrain to CURRENT/TORQUE only
                    if (flavor != VOCAB_CM_CURRENT && flavor != VOCAB_CM_TORQUE)
                    {
                        flavor = VOCAB_CM_TORQUE;
                    }
                    newActive = flavor;

                    // detect if the CST flavor changed since last time
                    flavourChanged
                        = (flavor != m_impl->controlModeState.prevCstFlavor[j])
                          || (m_impl->controlModeState.prevCstFlavor[j] == VOCAB_CM_UNKNOWN);
                } else
                {
                    newActive = Impl::ciaOpToYarp(activeOp);
                }
            }

            // If IDLE or FAULT or inhibited → clear SPs and latches; force target to IDLE on HW
            // inhibit
            if (newActive == VOCAB_CM_IDLE || newActive == VOCAB_CM_HW_FAULT
                || newActive == VOCAB_CM_FORCE_IDLE)
            {
                m_impl->setPoints.reset(j);
                m_impl->velLatched[j] = m_impl->trqLatched[j] = m_impl->posLatched[j]
                    = m_impl->currLatched[j] = false;
                if (hwInhibit)
                {
                    m_impl->controlModeState.target[j] = VOCAB_CM_IDLE;
                }
            }

            // Detect mode entry to (re)arm first-cycle latches
            if (m_impl->controlModeState.active[j] != newActive || flavourChanged)
            {
                // entering a control mode: arm latches and clear "has SP" flags
                m_impl->velLatched[j] = m_impl->trqLatched[j] = m_impl->posLatched[j]
                    = m_impl->currLatched[j] = false;
                m_impl->setPoints.reset(j);
            }

            m_impl->controlModeState.active[j] = newActive;
            m_impl->controlModeState.prevCstFlavor[j] = m_impl->controlModeState.cstFlavor[j];
        }
    }

    // ---- profiling: compute per-call duration and log 30s average ----
    constexpr double printingIntervalSec = 30.0;
    const auto tEnd = std::chrono::steady_clock::now();
    const double dtSec
        = std::chrono::duration_cast<std::chrono::duration<double>>(tEnd - tStart).count();
    m_impl->runTimeAccumSec += dtSec;
    m_impl->runTimeSamples += 1;
    const auto windowSec
        = std::chrono::duration_cast<std::chrono::duration<double>>(tEnd - m_impl->avgWindowStart)
              .count();
    if (windowSec >= printingIntervalSec)
    {
        const double avgMs
            = (m_impl->runTimeSamples > 0)
                  ? (m_impl->runTimeAccumSec / static_cast<double>(m_impl->runTimeSamples)) * 1000.0
                  : 0.0;
        yCInfoThrottle(CIA402,
                       printingIntervalSec,
                       "%s %.1fs window: avg run() time = %.3f ms (frequency=%.1f Hz) over %zu "
                       "cycles (window=%.1fs) - Expected period=%.3f ms (%.1f Hz)",
                       logPrefix,
                       printingIntervalSec,
                       avgMs,
                       (m_impl->runTimeSamples > 0) ? (1000.0 / avgMs) : 0.0,
                       m_impl->runTimeSamples,
                       windowSec,
                       this->getPeriod() * 1000.0,
                       1.0 / this->getPeriod());
        m_impl->runTimeAccumSec = 0.0;
        m_impl->runTimeSamples = 0;
        m_impl->avgWindowStart = tEnd;
    }
}

// -----------------------------------------------
// ----------------- IMotorEncoders --------------
// -----------------------------------------------

bool CiA402MotionControl::getNumberOfMotorEncoders(int* num)
{
    if (num == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *num = static_cast<int>(m_impl->numAxes);
    return true;
}

bool CiA402MotionControl::resetMotorEncoder(int m)
{
    yCError(CIA402, "%s: resetMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::resetMotorEncoders()
{
    yCError(CIA402, "%s: resetMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    yCError(CIA402,
            "%s: setMotorEncoderCountsPerRevolution() not implemented",
            Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoder(int m, const double v)
{
    yCError(CIA402, "%s: setMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoders(const double*)
{
    yCError(CIA402, "%s: setMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    if (cpr == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), m);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);

        // check which encoder is mounted on the motor side
        if (m_impl->enc1Mount[m] == Impl::Mount::Motor)
            *cpr = static_cast<double>(m_impl->enc1Res[m]);
        else if (m_impl->enc2Mount[m] == Impl::Mount::Motor)
            *cpr = static_cast<double>(m_impl->enc2Res[m]);
        else
            return false; // no encoder on motor side
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoder(int m, double* v)
{
    if (v == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *v = m_impl->variables.motorEncoders[m];
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoders(double* encs)
{
    if (encs == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(encs, m_impl->variables.motorEncoders.data(), m_impl->numAxes * sizeof(double));
    }

    return true;
}

bool CiA402MotionControl::getMotorEncodersTimed(double* encs, double* time)
{
    if (encs == nullptr || time == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(encs, m_impl->variables.motorEncoders.data(), m_impl->numAxes * sizeof(double));
        std::memcpy(time, m_impl->variables.feedbackTime.data(), m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoderTimed(int m, double* encs, double* time)
{
    if (encs == nullptr || time == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *encs = m_impl->variables.motorEncoders[m];
        *time = m_impl->variables.feedbackTime[m];
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoderSpeed(int m, double* sp)
{
    if (sp == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *sp = m_impl->variables.motorVelocities[m];
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoderSpeeds(double* spds)
{
    if (spds == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(spds,
                    m_impl->variables.motorVelocities.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoderAcceleration(int m, double* acc)
{
    if (acc == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *acc = m_impl->variables.motorAccelerations[m];
    }
    return true;
}

bool CiA402MotionControl::getMotorEncoderAccelerations(double* accs)
{
    if (accs == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(accs,
                    m_impl->variables.motorAccelerations.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

// -----------------------------------------------
// ----------------- IEncoders  ------------------
// -----------------------------------------------
bool CiA402MotionControl::getEncodersTimed(double* encs, double* time)
{
    if (encs == nullptr || time == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(encs,
                    m_impl->variables.jointPositions.data(),
                    m_impl->numAxes * sizeof(double));
        std::memcpy(time, m_impl->variables.feedbackTime.data(), m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getEncoderTimed(int j, double* encs, double* time)
{
    if (encs == nullptr || time == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *encs = m_impl->variables.jointPositions[j];
        *time = m_impl->variables.feedbackTime[j];
    }
    return true;
}

bool CiA402MotionControl::getAxes(int* ax)
{
    if (ax == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *ax = static_cast<int>(m_impl->numAxes);
    return true;
}

bool CiA402MotionControl::resetEncoder(int j)
{
    yCError(CIA402, "%s: resetEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::resetEncoders()
{
    yCError(CIA402, "%s: resetEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setEncoder(int j, const double val)
{
    yCError(CIA402, "%s: setEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setEncoders(const double* vals)
{
    yCError(CIA402, "%s: setEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getEncoder(int j, double* v)
{
    if (v == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *v = m_impl->variables.jointPositions[j];
    }
    return true;
}

bool CiA402MotionControl::getEncoders(double* encs)
{
    if (encs == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(encs,
                    m_impl->variables.jointPositions.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getEncoderSpeed(int j, double* sp)
{
    if (sp == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *sp = m_impl->variables.jointVelocities[j];
    }
    return true;
}

bool CiA402MotionControl::getEncoderSpeeds(double* spds)
{
    if (spds == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(spds,
                    m_impl->variables.jointVelocities.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getEncoderAcceleration(int j, double* spds)
{
    if (spds == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *spds = m_impl->variables.jointAccelerations[j];
    }
    return true;
}

bool CiA402MotionControl::getEncoderAccelerations(double* accs)
{
    if (accs == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(accs,
                    m_impl->variables.jointAccelerations.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

/// -----------------------------------------------
/// ----------------- IAxisInfo ------------------
/// -----------------------------------------------

bool CiA402MotionControl::getAxisName(int j, std::string& name)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // We assume that the name is already filled in the open() method and does not change
    // during the lifetime of the object. For this reason we do not need to lock the mutex
    // here.
    name = m_impl->variables.jointNames[j];
    return true;
}

bool CiA402MotionControl::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), axis);
        return false;
    }
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE; // TODO: add support for linear
                                                               // joints
    return true;
}

// -------------------------- IControlMode ------------------------------------
bool CiA402MotionControl::getControlMode(int j, int* mode)
{
    if (mode == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        *mode = m_impl->controlModeState.active[j];
    }
    return true;
}

bool CiA402MotionControl::getControlModes(int* modes)
{
    if (modes == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    {
        std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
        std::memcpy(modes, m_impl->controlModeState.active.data(), m_impl->numAxes * sizeof(int));
    }
    return true;
}

bool CiA402MotionControl::getControlModes(const int n, const int* joints, int* modes)
{
    if (modes == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n);
        return false;
    }

    {
        std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
        for (int k = 0; k < n; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            modes[k] = m_impl->controlModeState.active[joints[k]];
        }
    }
    return true;
}

bool CiA402MotionControl::setControlMode(const int j, const int mode)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    if (Impl::yarpToCiaOp(mode) < 0)
    {
        yCError(CIA402, "%s: control mode %d not supported", Impl::kClassName.data(), mode);
        return false;
    }

    if (mode == VOCAB_CM_POSITION_DIRECT)
    {
        yCError(CIA402,
                "%s: control mode %d (POSITION_DIRECT) not supported",
                Impl::kClassName.data(),
                mode);
        return false;
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    m_impl->controlModeState.target[j] = mode;
    if (mode == VOCAB_CM_CURRENT || mode == VOCAB_CM_TORQUE)
    {
        m_impl->controlModeState.cstFlavor[j] = mode;
    }

    return true;
}

bool CiA402MotionControl::setControlModes(const int n, const int* joints, int* modes)
{
    if (modes == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n);
        return false;
    }

    // check if one of the modes is VOCAB_CM_POSITION_DIRECT
    for (int k = 0; k < n; ++k)
    {
        if (modes[k] == VOCAB_CM_POSITION_DIRECT)
        {
            yCError(CIA402,
                    "%s: control mode %d (POSITION_DIRECT) not supported",
                    Impl::kClassName.data(),
                    modes[k]);
            return false;
        }
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    for (int k = 0; k < n; ++k)
    {
        if (joints[k] >= static_cast<int>(m_impl->numAxes))
        {
            yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
            return false;
        }
        m_impl->controlModeState.target[joints[k]] = modes[k];

        if (modes[k] == VOCAB_CM_CURRENT || modes[k] == VOCAB_CM_TORQUE)
        {
            m_impl->controlModeState.cstFlavor[joints[k]] = modes[k];
        }
    }
    return true;
}

bool CiA402MotionControl::setControlModes(int* modes)
{
    if (modes == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    // check if one of the modes is VOCAB_CM_POSITION_DIRECT
    for (int k = 0; k < m_impl->numAxes; ++k)
    {
        if (modes[k] == VOCAB_CM_POSITION_DIRECT)
        {
            yCError(CIA402,
                    "%s: control mode %d (POSITION_DIRECT) not supported",
                    Impl::kClassName.data(),
                    modes[k]);
            return false;
        }
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    std::memcpy(m_impl->controlModeState.target.data(), modes, m_impl->numAxes * sizeof(int));

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        if (m_impl->controlModeState.target[j] == VOCAB_CM_CURRENT
            || m_impl->controlModeState.target[j] == VOCAB_CM_TORQUE)
        {
            m_impl->controlModeState.cstFlavor[j] = m_impl->controlModeState.target[j];
        }
    }

    return true;
}

//// -------------------------- ITorqueControl ------------------------------------
bool CiA402MotionControl::getTorque(int j, double* t)
{
    if (t == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        *t = m_impl->variables.jointTorques[j];
    }
    return true;
}

bool CiA402MotionControl::getTorques(double* t)
{
    if (t == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);

        std::memcpy(t, m_impl->variables.jointTorques.data(), m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getRefTorque(int j, double* t)
{
    if (t == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        *t = m_impl->setPoints.jointTorques[j];
    }
    return true;
}

bool CiA402MotionControl::getRefTorques(double* t)
{
    if (t == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        std::memcpy(t, m_impl->setPoints.jointTorques.data(), m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::setRefTorque(int j, double t)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // (a/b) Only accept if TORQUE is ACTIVE; otherwise reject (not considered)
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[j] != VOCAB_CM_TORQUE)
        {
            yCError(CIA402,
                    "%s: setRefTorque rejected: TORQUE mode is not active for the joint %d",
                    Impl::kClassName.data(),
                    j);
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    m_impl->setPoints.jointTorques[j] = t;
    m_impl->setPoints.hasTorqueSP[j] = true; // (b)
    return true;
}

bool CiA402MotionControl::setRefTorques(const double* t)
{
    if (t == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        // check that all the joints are in TORQUE mode use std function without for loop
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] != VOCAB_CM_TORQUE)
            {
                yCError(CIA402,
                        "%s: setRefTorques rejected: TORQUE mode is not active for the joint "
                        "%zu",
                        Impl::kClassName.data(),
                        j);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    std::memcpy(m_impl->setPoints.jointTorques.data(), t, m_impl->numAxes * sizeof(double));
    std::fill(m_impl->setPoints.hasTorqueSP.begin(), m_impl->setPoints.hasTorqueSP.end(), true);
    return true;
}

bool CiA402MotionControl::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    if (t == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // check that all the joints are in TORQUE mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_TORQUE)
            {
                yCError(CIA402,
                        "%s: setRefTorques rejected: TORQUE mode is not active for the joint %d",
                        Impl::kClassName.data(),
                        joints[k]);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n_joint; ++k)
    {
        m_impl->setPoints.jointTorques[joints[k]] = t[k];
        m_impl->setPoints.hasTorqueSP[joints[k]] = true;
    }
    return true;
}

bool CiA402MotionControl::getTorqueRange(int j, double* min, double* max)
{
    if (min == nullptr || max == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // multiply by the transmission ratio to convert motor torque to joint torque
    *min = -m_impl->maxMotorTorqueNm[j] * m_impl->gearRatio[j];
    *max = m_impl->maxMotorTorqueNm[j] * m_impl->gearRatio[j];

    return true;
}

bool CiA402MotionControl::getTorqueRanges(double* min, double* max)
{
    if (min == nullptr || max == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        min[j] = -m_impl->maxMotorTorqueNm[j] * m_impl->gearRatio[j];
        max[j] = m_impl->maxMotorTorqueNm[j] * m_impl->gearRatio[j];
    }
    return true;
}

bool CiA402MotionControl::velocityMove(int j, double spd)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // (a/b) Only accept if VELOCITY is ACTIVE; otherwise reject
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[j] != VOCAB_CM_VELOCITY)
        {
            yCError(CIA402,
                    "%s: velocityMove rejected: VELOCITY mode is not active for the joint %d",
                    Impl::kClassName.data(),
                    j);

            // this will return true to indicate the rejection was handled
            return true;
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    m_impl->setPoints.jointVelocities[j] = spd;
    m_impl->setPoints.hasVelSP[j] = true; // (b)
    return true;
}

bool CiA402MotionControl::velocityMove(const double* spds)
{
    if (spds == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    // check that all the joints are in VELOCITY mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] != VOCAB_CM_VELOCITY)
            {
                yCError(CIA402,
                        "%s: velocityMove rejected: VELOCITY mode is not active for the joint "
                        "%zu",
                        Impl::kClassName.data(),
                        j);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    std::memcpy(m_impl->setPoints.jointVelocities.data(), spds, m_impl->numAxes * sizeof(double));
    std::fill(m_impl->setPoints.hasVelSP.begin(), m_impl->setPoints.hasVelSP.end(), true);
    return true;
}

bool CiA402MotionControl::getRefVelocity(const int joint, double* vel)
{
    if (vel == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (joint >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joint);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        *vel = m_impl->setPoints.jointVelocities[joint];
    }
    return true;
}

bool CiA402MotionControl::getRefVelocities(double* spds)
{
    if (spds == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        std::memcpy(spds,
                    m_impl->setPoints.jointVelocities.data(),
                    m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool CiA402MotionControl::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    if (vels == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            vels[k] = m_impl->setPoints.jointVelocities[joints[k]];
        }
    }
    return true;
}

// Acceleration is expressed in YARP joint units [deg/s^2].
// For PP we SDO-write 0x6083/0x6084/0x6085 in rpm/s (deg/s^2 ÷ 6).
// For CSV we just cache the value (no standard accel SDO in CSV).

bool CiA402MotionControl::setRefAcceleration(int j, double accDegS2)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: setRefAcceleration: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // store magnitude
    if (accDegS2 < 0.0)
    {
        accDegS2 = -accDegS2;
    }

    // Only touch SDOs if PP is ACTIVE
    int controlMode = -1;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        controlMode = m_impl->controlModeState.active[j];
    }

    if (controlMode != VOCAB_CM_POSITION)
    {
        // do nothing if not in PP
        return true;
    }

    const int s = m_impl->firstSlave + j;
    const int32_t acc = static_cast<int32_t>(std::llround(accDegS2 * m_impl->degSToVel[j]));
    {
        std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
        m_impl->ppState.ppRefAccelerationDegSS[j] = accDegS2;
    }

    // Profile acceleration / deceleration / quick-stop deceleration
    (void)m_impl->ethercatManager.writeSDO<int32_t>(s, 0x6083, 0x00, acc);
    (void)m_impl->ethercatManager.writeSDO<int32_t>(s, 0x6084, 0x00, acc);
    (void)m_impl->ethercatManager.writeSDO<int32_t>(s, 0x6085, 0x00, acc);

    return true;
}

bool CiA402MotionControl::setRefAccelerations(const double* accsDegS2)
{
    if (!accsDegS2)
    {
        yCError(CIA402, "%s: setRefAccelerations: null pointer", Impl::kClassName.data());
        return false;
    }
    bool ok = true;
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        ok = setRefAcceleration(static_cast<int>(j), accsDegS2[j]) && ok;
    }
    return ok;
}

bool CiA402MotionControl::getRefAcceleration(int j, double* acc)
{
    if (acc == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    int controlMode = -1;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        controlMode = m_impl->controlModeState.active[j];
    }

    if (controlMode == VOCAB_CM_POSITION)
    {
        // if in PP return the cached value
        std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
        *acc = m_impl->ppState.ppRefAccelerationDegSS[j];
        return true;
    }

    *acc = 0.0;
    return true;
}

bool CiA402MotionControl::getRefAccelerations(double* accs)
{
    if (accs == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        int controlMode = -1;
        {
            std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
            controlMode = m_impl->controlModeState.active[j];
        }

        if (controlMode == VOCAB_CM_POSITION)
        {
            // if in PP return the cached value
            std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
            accs[j] = m_impl->ppState.ppRefAccelerationDegSS[j];
        } else
        {
            accs[j] = 0.0;
        }
    }
    return true;
}

// stop() semantics:
//  • PP → set CW bit 8 (HALT), decelerating with 0x6084.
//  • CSV/others → zero TargetVelocity (current behavior kept).

bool CiA402MotionControl::stop(int j)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: stop: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    int controlMode = -1;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        controlMode = m_impl->controlModeState.active[j];
    }

    if (controlMode == VOCAB_CM_POSITION)
    {
        std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
        m_impl->ppState.ppHaltRequested[j] = true; // consumed in setSetPoints()
        return true;
    }

    // CSV/others → zero velocity set-point
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        m_impl->setPoints.jointVelocities[j] = 0.0;
        m_impl->setPoints.hasVelSP[j] = true;
    }
    return true;
}

bool CiA402MotionControl::stop()
{
    // PP axes → HALT; non-PP axes → velocity 0
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] == VOCAB_CM_POSITION)
            {
                std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
                m_impl->ppState.ppHaltRequested[j] = true;
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        std::fill(m_impl->setPoints.jointVelocities.begin(),
                  m_impl->setPoints.jointVelocities.end(),
                  0.0);
        std::fill(m_impl->setPoints.hasVelSP.begin(), m_impl->setPoints.hasVelSP.end(), true);
    }
    return true;
}

bool CiA402MotionControl::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    if (spds == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // check that all the joints are in VELOCITY mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_VELOCITY)
            {
                yCError(CIA402,
                        "%s: velocityMove rejected: VELOCITY mode is not active for the joint "
                        "%d",
                        Impl::kClassName.data(),
                        joints[k]);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n_joint; ++k)
    {
        m_impl->setPoints.jointVelocities[joints[k]] = spds[k];
        m_impl->setPoints.hasVelSP[joints[k]] = true;
    }

    return true;
}

bool CiA402MotionControl::setRefAccelerations(const int n_joint,
                                              const int* joints,
                                              const double* accs)
{
    if (accs == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // no operation
    return true;
}

bool CiA402MotionControl::getRefAccelerations(const int n_joint, const int* joints, double* accs)
{
    if (accs == nullptr || joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    std::memset(accs, 0, n_joint * sizeof(double)); // CiA-402 does not support acceleration
                                                    // setpoints, so we return 0.0 for all axes
    return true;
}

bool CiA402MotionControl::stop(const int n_joint, const int* joints)
{
    if (joints == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yCError(CIA402, "%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            m_impl->setPoints.jointVelocities[joints[k]] = 0.0;
            m_impl->setPoints.hasVelSP[joints[k]] = true;
        }
    }
    return true;
}

bool CiA402MotionControl::getLastJointFault(int j, int& fault, std::string& message)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getLastJointFault: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    const int slave = m_impl->firstSlave + j;

    // --- 0x603F:00 Error code (UINT16) ---
    uint16_t code = 0;
    auto err = m_impl->ethercatManager.readSDO<uint16_t>(slave, 0x603F, 0x00, code);
    if (err != ::CiA402::EthercatManager::Error::NoError)
    {
        yCError(CIA402,
                "%s: getLastJointFault: SDO read 0x603F:00 failed (joint %d)",
                Impl::kClassName.data(),
                j);
        return false;
    }

    fault = static_cast<int>(code);
    message = m_impl->describe603F(code);

    // --- 0x203F:01 Error report (STRING(8)) ---
    // Use a fixed char[8] so we can call the templated readSDO<T> unmodified.
    char report[8] = {0}; // zero-init so partial reads are safely NUL-terminated
    auto err2 = m_impl->ethercatManager.readSDO(slave, 0x203F, 0x01, report);
    if (err2 == ::CiA402::EthercatManager::Error::NoError)
    {
        // Trim at first NUL (STRING(8) is not guaranteed to be fully used)
        std::size_t len = 0;
        while (len < sizeof(report) && report[len] != '\0')
            ++len;

        if (len > 0)
        {
            // If it's clean printable ASCII, append as text; otherwise show hex bytes.
            bool printable = true;
            for (std::size_t i = 0; i < len; ++i)
            {
                const unsigned char c = static_cast<unsigned char>(report[i]);
                if (c < 0x20 || c > 0x7E)
                {
                    printable = false;
                    break;
                }
            }

            if (printable)
            {
                message += " — report: ";
                message.append(report, len);
            } else
            {
                char hex[3 * 8 + 1] = {0};
                int pos = 0;
                for (std::size_t i = 0; i < len && pos <= static_cast<int>(sizeof(hex)) - 3; ++i)
                    pos += std::snprintf(hex + pos,
                                         sizeof(hex) - pos,
                                         "%02X%s",
                                         static_cast<unsigned char>(report[i]),
                                         (i + 1 < len) ? " " : "");
                message += " — report: [";
                message += hex;
                message += "]";
            }
        }
    }
    return true;
}

bool CiA402MotionControl::positionMove(int j, double refDeg)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: positionMove: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    // accept only if PP is ACTIVE (same policy as your CSV path)
    int controlMode = -1;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        controlMode = m_impl->controlModeState.active[j];
    }

    if (controlMode != VOCAB_CM_POSITION)
    {
        yCError(CIA402,
                "%s: positionMove rejected: POSITION mode not active for joint %d",
                Impl::kClassName.data(),
                j);
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    m_impl->setPoints.jointPositionsDeg[j] = refDeg;
    m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(size_t(j), refDeg);
    m_impl->setPoints.isRelative[j] = false;
    m_impl->setPoints.hasPosSP[j] = true;
    m_impl->setPoints.pulseHi[j] = true; // schedule rising edge on CW bit4
    return true;
}

bool CiA402MotionControl::positionMove(const double* refsDeg)
{
    if (!refsDeg)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    // all axes must be in PP
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
            if (m_impl->controlModeState.active[j] != VOCAB_CM_POSITION)
            {
                yCError(CIA402,
                        "%s: positionMove rejected: POSITION mode not active on joint %zu",
                        Impl::kClassName.data(),
                        j);
                return false;
            }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->setPoints.jointPositionsDeg[j] = refsDeg[j];
        m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(j, refsDeg[j]);
        m_impl->setPoints.isRelative[j] = false;
        m_impl->setPoints.hasPosSP[j] = true;
        m_impl->setPoints.pulseHi[j] = true;
    }
    return true;
}

bool CiA402MotionControl::positionMove(const int n, const int* joints, const double* refsDeg)
{
    if (!joints || !refsDeg || n <= 0)
    {
        yCError(CIA402, "%s: invalid args", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        for (int k = 0; k < n; ++k)
        {
            if (joints[k] < 0 || joints[k] >= static_cast<int>(m_impl->numAxes))
                return false;
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_POSITION)
            {
                yCError(CIA402,
                        "%s: positionMove rejected: POSITION mode not active on joint %d",
                        Impl::kClassName.data(),
                        joints[k]);
                return false;
            }
        }
    }
    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n; ++k)
    {
        const int j = joints[k];
        m_impl->setPoints.jointPositionsDeg[j] = refsDeg[k];
        m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(size_t(j), refsDeg[k]);
        m_impl->setPoints.isRelative[j] = false;
        m_impl->setPoints.hasPosSP[j] = true;
        m_impl->setPoints.pulseHi[j] = true;
    }
    return true;
}

bool CiA402MotionControl::relativeMove(int j, double deltaDeg)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
        return false;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[j] != VOCAB_CM_POSITION)
        {
            yCError(CIA402,
                    "%s: relativeMove rejected: POSITION mode not active for joint %d",
                    Impl::kClassName.data(),
                    j);
            return true;
        }
    }
    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    m_impl->setPoints.jointPositionsDeg[j] += deltaDeg; // cache last target in deg
    m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(size_t(j), deltaDeg);
    m_impl->setPoints.isRelative[j] = true;
    m_impl->setPoints.hasPosSP[j] = true;
    m_impl->setPoints.pulseHi[j] = true;
    return true;
}

bool CiA402MotionControl::relativeMove(const double* deltasDeg)
{
    if (!deltasDeg)
        return false;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
            if (m_impl->controlModeState.active[j] != VOCAB_CM_POSITION)
                return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->setPoints.jointPositionsDeg[j] += deltasDeg[j];
        m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(j, deltasDeg[j]);
        m_impl->setPoints.isRelative[j] = true;
        m_impl->setPoints.hasPosSP[j] = true;
        m_impl->setPoints.pulseHi[j] = true;
    }
    return true;
}

bool CiA402MotionControl::relativeMove(const int n, const int* joints, const double* deltasDeg)
{
    if (!joints || !deltasDeg || n <= 0)
        return false;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        for (int k = 0; k < n; ++k)
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_POSITION)
                return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n; ++k)
    {
        const int j = joints[k];
        m_impl->setPoints.jointPositionsDeg[j] += deltasDeg[k];
        m_impl->setPoints.targetCounts[j] = m_impl->jointDegToTargetCounts(size_t(j), deltasDeg[k]);
        m_impl->setPoints.isRelative[j] = true;
        m_impl->setPoints.hasPosSP[j] = true;
        m_impl->setPoints.pulseHi[j] = true;
    }
    return true;
}

bool CiA402MotionControl::checkMotionDone(int j, bool* flag)
{
    if (flag == nullptr)
    {
        yCError(CIA402, "%s: checkMotionDone: null pointer", Impl::kClassName.data());
        return false;
    }

    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: checkMotionDone: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    *flag = m_impl->variables.targetReached[j];
    return true;
}

bool CiA402MotionControl::checkMotionDone(bool* flag)
{
    if (flag == nullptr)
    {
        yCError(CIA402, "%s: checkMotionDone: null pointer", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        flag[j] = m_impl->variables.targetReached[j];
    }

    return true;
}

bool CiA402MotionControl::checkMotionDone(const int n, const int* joints, bool* flag)
{
    if (!joints || !flag || n <= 0)
        return false;
    bool all = true;
    for (int k = 0; k < n; ++k)
    {
        bool f = true;
        checkMotionDone(joints[k], &f);
        all = all && f;
    }
    *flag = all;
    return true;
}

bool CiA402MotionControl::setRefSpeed(int j, double spDegS)
{
    if (j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: setRefSpeed: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // Only write SDO if PP is active (current behavior)
    int cm = -1;
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        cm = m_impl->controlModeState.active[j];
    }
    if (cm != VOCAB_CM_POSITION)
    {
        yCError(CIA402,
                "%s: setRefSpeed: POSITION mode not active for joint %d, not writing SDO",
                Impl::kClassName.data(),
                j);
        return true;
    }

    // Cache for getRefSpeed()
    {
        std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
        m_impl->ppState.ppRefSpeedDegS[j] = spDegS;
    }

    m_impl->setSDORefSpeed(j, spDegS);

    return true;
}

bool CiA402MotionControl::setRefSpeeds(const double* spDegS)
{
    if (spDegS == nullptr)
    {
        yCError(CIA402, "%s: setRefSpeeds: null pointer", Impl::kClassName.data());
        return false;
    }
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        this->setRefSpeed(static_cast<int>(j), spDegS[j]);
    }
    return true;
}

bool CiA402MotionControl::setRefSpeeds(const int n, const int* joints, const double* spDegS)
{
    if (!joints || !spDegS || n <= 0)
    {
        yCError(CIA402, "%s: setRefSpeeds: invalid args", Impl::kClassName.data());
        return false;
    }

    for (int k = 0; k < n; ++k)
    {
        this->setRefSpeed(joints[k], spDegS[k]);
    }
    return true;
}

bool CiA402MotionControl::getRefSpeed(int j, double* ref)
{
    if (!ref || j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getRefSpeed: invalid args", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
    *ref = m_impl->ppState.ppRefSpeedDegS[j];
    return true;
}
bool CiA402MotionControl::getRefSpeeds(double* spds)
{
    if (!spds)
    {
        yCError(CIA402, "%s: getRefSpeeds: null pointer", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
    std::memcpy(spds, m_impl->ppState.ppRefSpeedDegS.data(), m_impl->numAxes * sizeof(double));
    return true;
}

bool CiA402MotionControl::getRefSpeeds(const int n, const int* joints, double* spds)
{
    if (!joints || !spds || n <= 0)
    {
        yCError(CIA402, "%s: getRefSpeeds: invalid args", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->ppState.mutex);
    for (int k = 0; k < n; ++k)
    {
        spds[k] = m_impl->ppState.ppRefSpeedDegS[joints[k]];
    }
    return true;
}

bool CiA402MotionControl::getTargetPosition(const int j, double* ref)
{
    if (!ref || j < 0 || j >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getTargetPosition: invalid args", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    *ref = m_impl->setPoints.jointPositionsDeg[j];
    return true;
}

bool CiA402MotionControl::getTargetPositions(double* refs)
{
    if (refs == nullptr)
    {
        yCError(CIA402, "%s: getTargetPositions: null pointer", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    std::memcpy(refs, m_impl->setPoints.jointPositionsDeg.data(), m_impl->numAxes * sizeof(double));
    return true;
}

bool CiA402MotionControl::getTargetPositions(const int n, const int* joints, double* refs)
{
    if (!joints || !refs || n <= 0)
    {
        yCError(CIA402, "%s: getTargetPositions: invalid args", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n; ++k)
    {
        refs[k] = m_impl->setPoints.jointPositionsDeg[joints[k]];
    }
    return true;
}

bool CiA402MotionControl::getNumberOfMotors(int* num)
{
    if (!num)
    {
        yCError(CIA402, "%s: getNumberOfMotors: null pointer", Impl::kClassName.data());
        return false;
    }
    *num = m_impl->numAxes;
    return true;
}

bool CiA402MotionControl::getTemperature(int m, double* val)
{
    if (val == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    *val = m_impl->variables.driveTemperatures[m];
    return true;
}

bool CiA402MotionControl::getTemperatures(double* vals)
{
    if (vals == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    std::memcpy(vals, m_impl->variables.driveTemperatures.data(), m_impl->numAxes * sizeof(double));
    return true;
}

bool CiA402MotionControl::getTemperatureLimit(int m, double* temp)
{
    // The get temperature limit function is not implemented
    yCError(CIA402,
            "%s: The getTemperatureLimit function is not implemented",
            Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setTemperatureLimit(int m, const double temp)
{
    // The set temperature limit function is not implemented
    yCError(CIA402,
            "%s: The setTemperatureLimit function is not implemented",
            Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getGearboxRatio(int m, double* val)
{
    if (val == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    *val = m_impl->gearRatio[m];
    return true;
}

bool CiA402MotionControl::setGearboxRatio(int m, const double val)
{
    // The setGearboxRatio function is not implemented
    yCError(CIA402, "%s: The setGearboxRatio function is not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getCurrent(int m, double* curr)
{
    if (curr == nullptr)
    {
        yCError(CIA402, "%s: getCurrent: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getCurrent: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    *curr = m_impl->variables.motorCurrents[m];
    return true;
};

bool CiA402MotionControl::getCurrents(double* currs)
{
    if (currs == nullptr)
    {
        yCError(CIA402, "%s: getCurrents: null pointer", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    std::memcpy(currs, m_impl->variables.motorCurrents.data(), m_impl->numAxes * sizeof(double));
    return true;
}

bool CiA402MotionControl::getCurrentRange(int m, double* min, double* max)
{
    if (min == nullptr || max == nullptr)
    {
        yCError(CIA402, "%s: getCurrentRange: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getCurrentRange: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    *min = -m_impl->maxCurrentsA[m];
    *max = m_impl->maxCurrentsA[m];
    return true;
}

bool CiA402MotionControl::getCurrentRanges(double* min, double* max)
{
    if (min == nullptr || max == nullptr)
    {
        yCError(CIA402, "%s: getCurrentRanges: null pointer", Impl::kClassName.data());
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
    for (size_t m = 0; m < m_impl->numAxes; ++m)
    {
        min[m] = -m_impl->maxCurrentsA[m];
        max[m] = m_impl->maxCurrentsA[m];
    }
    return true;
}

bool CiA402MotionControl::setRefCurrents(const double* currs)
{
    if (currs == nullptr)
    {
        yCError(CIA402, "%s: setRefCurrents: null pointer", Impl::kClassName.data());
        return false;
    }

    {
        // check that all the joints are in CURRENT mode
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] != VOCAB_CM_CURRENT)
            {
                yCError(CIA402,
                        "%s: setRefCurrents rejected: CURRENT mode is not active for the joint "
                        "%zu",
                        Impl::kClassName.data(),
                        j);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    std::memcpy(m_impl->setPoints.motorCurrents.data(), currs, m_impl->numAxes * sizeof(double));
    std::fill(m_impl->setPoints.hasCurrentSP.begin(), m_impl->setPoints.hasCurrentSP.end(), true);
    return true;
}

bool CiA402MotionControl::setRefCurrent(int m, double curr)
{
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: setRefCurrent: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }

    // (a/b) Only accept if CURRENT is ACTIVE; otherwise reject (not considered)
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[m] != VOCAB_CM_CURRENT)
        {
            yCError(CIA402,
                    "%s: setRefCurrent rejected: CURRENT mode is not active for the joint %d",
                    Impl::kClassName.data(),
                    m);
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    m_impl->setPoints.motorCurrents[m] = curr;
    m_impl->setPoints.hasCurrentSP[m] = true; // (b)

    return true;
}

bool CiA402MotionControl::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    if (currs == nullptr || motors == nullptr)
    {
        yCError(CIA402, "%s: setRefCurrents: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_motor <= 0)
    {
        yCError(CIA402,
                "%s: setRefCurrents: invalid number of motors %d",
                Impl::kClassName.data(),
                n_motor);
        return false;
    }
    for (int k = 0; k < n_motor; ++k)
    {
        if (motors[k] < 0 || motors[k] >= static_cast<int>(m_impl->numAxes))
        {
            yCError(CIA402,
                    "%s: setRefCurrents: motor %d out of range",
                    Impl::kClassName.data(),
                    motors[k]);
            return false;
        }
    }

    // check that all the joints are in CURRENT mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (int k = 0; k < n_motor; ++k)
        {
            if (m_impl->controlModeState.active[motors[k]] != VOCAB_CM_CURRENT)
            {
                yCError(CIA402,
                        "%s: setRefCurrents rejected: CURRENT mode is not active for the joint %d",
                        Impl::kClassName.data(),
                        motors[k]);
                return false; // reject
            }
        }
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    for (int k = 0; k < n_motor; ++k)
    {
        m_impl->setPoints.motorCurrents[motors[k]] = currs[k];
        m_impl->setPoints.hasCurrentSP[motors[k]] = true;
    }
    return true;
}

bool CiA402MotionControl::getRefCurrents(double* currs)
{
    if (currs == nullptr)
    {
        yCError(CIA402, "%s: getRefCurrents: null pointer", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    std::memcpy(currs, m_impl->setPoints.motorCurrents.data(), m_impl->numAxes * sizeof(double));
    return true;
}

bool CiA402MotionControl::getRefCurrent(int m, double* curr)
{
    if (curr == nullptr)
    {
        yCError(CIA402, "%s: getRefCurrent: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m < 0 || m >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getRefCurrent: motor %d out of range", Impl::kClassName.data(), m);
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
    *curr = m_impl->setPoints.motorCurrents[m];
    return true;
}

bool CiA402MotionControl::setLimits(int axis, double min, double max)
{
    if (axis < 0 || axis >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: setLimits: axis %d out of range", Impl::kClassName.data(), axis);
        return false;
    }
    // If both bounds are provided (non-negative), enforce min < max.
    // When either bound is negative, we treat that side as disabled like in open(),
    // and skip the ordering check.
    if (!(min < 0.0 || max < 0.0) && (min >= max))
    {
        yCError(CIA402,
                "%s: setLimits: invalid limits [min=%g, max=%g]",
                Impl::kClassName.data(),
                min,
                max);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->limits.mutex);
        m_impl->limits.maxPositionLimitDeg[axis] = max;
        m_impl->limits.minPositionLimitDeg[axis] = min;
    }

    // set the software limits in the drive (SDO 0x607D)
    // convert the limits from deg to counts, honoring inversion and disabled bounds
    const bool inv = m_impl->invertedMotionSenseDirection[axis];
    const bool lowerLimitDisabled = (min < 0.0);
    const bool upperLimitDisabled = (max < 0.0);

    const auto lowerLimitCounts
        = lowerLimitDisabled ? 0 : m_impl->jointDegToTargetCounts(size_t(axis), min);
    const auto upperLimitCounts
        = upperLimitDisabled ? 0 : m_impl->jointDegToTargetCounts(size_t(axis), max);

    int32_t minCounts = 0;
    int32_t maxCounts = 0;

    if (!inv)
    {
        minCounts = lowerLimitDisabled ? std::numeric_limits<int32_t>::min() : lowerLimitCounts;
        maxCounts = upperLimitDisabled ? std::numeric_limits<int32_t>::max() : upperLimitCounts;
    } else
    {
        // Inverted: f(x) = -counts(x), so [min,max] -> [f(max), f(min)]
        minCounts = upperLimitDisabled ? std::numeric_limits<int32_t>::min() : -upperLimitCounts;
        maxCounts = lowerLimitDisabled ? std::numeric_limits<int32_t>::max() : -lowerLimitCounts;
    }

    return m_impl->setPositionCountsLimits(axis, minCounts, maxCounts);
}

bool CiA402MotionControl::getLimits(int axis, double* min, double* max)
{
    if (min == nullptr || max == nullptr)
    {
        yCError(CIA402, "%s: getLimits: null pointer", Impl::kClassName.data());
        return false;
    }
    if (axis < 0 || axis >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: getLimits: axis %d out of range", Impl::kClassName.data(), axis);
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->limits.mutex);
    *min = m_impl->limits.minPositionLimitDeg[axis];
    *max = m_impl->limits.maxPositionLimitDeg[axis];
    return true;
}

bool CiA402MotionControl::setVelLimits(int axis, double min, double max)
{
    // not implemented yet
    constexpr auto logPrefix = "[setVelLimits] ";
    yCError(CIA402, "%s: The setVelLimits function is not implemented", logPrefix);
    return false;
}

bool CiA402MotionControl::getVelLimits(int axis, double* min, double* max)
{
    // not implemented yet
    constexpr auto logPrefix = "[getVelLimits] ";
    yCError(CIA402, "%s: The getVelLimits function is not implemented", logPrefix);
    return false;
}

bool CiA402MotionControl::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    if (!mode)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    *mode = m_impl->dummyInteractionMode;
    return true;
}

bool CiA402MotionControl::getInteractionModes(int n_joints,
                                              int* joints,
                                              yarp::dev::InteractionModeEnum* modes)
{
    if (!joints || !modes || n_joints <= 0)
    {
        yCError(CIA402, "%s: invalid args", Impl::kClassName.data());
        return false;
    }

    for (int k = 0; k < n_joints; ++k)
    {
        if (joints[k] < 0 || joints[k] >= static_cast<int>(m_impl->numAxes))
        {
            yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
            return false;
        }
        modes[k] = m_impl->dummyInteractionMode;
    }
    return true;
}

bool CiA402MotionControl::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (modes == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        modes[j] = m_impl->dummyInteractionMode;
    }
    return true;
}

bool CiA402MotionControl::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    if (axis < 0 || axis >= static_cast<int>(m_impl->numAxes))
    {
        yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), axis);
        return false;
    }

    // The interaction mode is not implemented in this driver.
    yCError(CIA402,
            "%s: The setInteractionMode function is not implemented",
            Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setInteractionModes(int n_joints,
                                              int* joints,
                                              yarp::dev::InteractionModeEnum* modes)
{
    if (!joints || !modes || n_joints <= 0)
    {
        yCError(CIA402, "%s: invalid args", Impl::kClassName.data());
        return false;
    }

    for (int k = 0; k < n_joints; ++k)
    {
        if (joints[k] < 0 || joints[k] >= static_cast<int>(m_impl->numAxes))
        {
            yCError(CIA402, "%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
            return false;
        }
    }

    // The interaction mode is not implemented in this driver.
    yCError(CIA402,
            "%s: The setInteractionModes function is not implemented",
            Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (modes == nullptr)
    {
        yCError(CIA402, "%s: null pointer", Impl::kClassName.data());
        return false;
    }

    yCError(CIA402,
            "%s: The setInteractionModes function is not implemented",
            Impl::kClassName.data());
    return false;
}

} // namespace yarp::dev

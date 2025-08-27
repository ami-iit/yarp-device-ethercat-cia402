// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "CiA402MotionControl.h"
#include "CiA402StateMachine.h"
#include "EthercatManager.h"

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>


// STD
#include <algorithm>
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

    // Constants (unit conversions used throughout)
    static constexpr double RPM_TO_DEG_PER_SEC = 360.0 / 60.0; // rpm → deg/s
    static constexpr double DEG_PER_SEC_TO_RPM = 60.0 / 360.0; // deg/s → rpm
    static constexpr double MICROSECONDS_TO_SECONDS = 1e-6; // µs → s

    // SOEM needs one contiguous memory area for every byte that travels on the bus.
    char ioMap[4096] = {0};

    //--------------------------------------------------------------------------
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
        std::vector<double> feedbackTime; // feedback time in seconds
        std::vector<uint8_t> STO;
        std::vector<uint8_t> SBC;
        std::vector<std::string> jointNames; // for getAxisName()

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
            this->STO.resize(numAxes);
            this->SBC.resize(numAxes);
        }
    };

    VariablesReadFromMotors variables;

    struct ControlModeState
    {
        std::vector<int> target; // what the user asked for
        std::vector<int> active; // what the drive is really doing
        mutable std::mutex mutex; // protects *both* vectors

        void resize(std::size_t n, int initial = VOCAB_CM_IDLE)
        {
            target.assign(n, initial);
            active = target;
        }
    };
    ControlModeState controlModeState;

    struct SetPoints
    {
        std::mutex mutex; // protects the following vectors
        std::vector<double> jointTorques; // for setTorque()
        std::vector<double> jointVelocities; // for velocityMove() or joint velocity commands
        std::vector<bool> hasTorqueSP; // user provided a torque since entry?
        std::vector<bool> hasVelSP; // user provided a velocity since entry?

        void resize(std::size_t n)
        {
            jointTorques.resize(n);
            jointVelocities.resize(n);
            hasTorqueSP.assign(n, false);
            hasVelSP.assign(n, false);
        }

        void reset()
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            std::fill(this->jointTorques.begin(), this->jointTorques.end(), 0.0);
            std::fill(this->jointVelocities.begin(), this->jointVelocities.end(), 0.0);
            std::fill(this->hasTorqueSP.begin(), this->hasTorqueSP.end(), false);
            std::fill(this->hasVelSP.begin(), this->hasVelSP.end(), false);
        }

        void reset(int axis)
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->jointTorques[axis] = 0.0;
            this->jointVelocities[axis] = 0.0;
            this->hasTorqueSP[axis] = false;
            this->hasVelSP[axis] = false;
        }
    };

    SetPoints setPoints;

    /* --------------- CiA-402 power-state machine ------------------------ */
    std::vector<std::unique_ptr<CiA402::StateMachine>> sm;

    // First-cycle latches and seed
    std::vector<bool> velLatched, trqLatched;
    std::vector<double> torqueSeedNm;
    std::vector<int> lastActiveMode;

    bool opRequested{false}; // true after the first run() call

    Impl() = default;
    ~Impl() = default;

   
    void fillJointNames()
    {
        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);
            this->variables.jointNames[j] = ethercatManager.getName(slaveIdx);
        }
    }

    //--------------------------------------------------------------------------
    //  One-shot SDO reads – here we only fetch encoder resolutions
    //--------------------------------------------------------------------------
    bool readEncoderResolutions()
    {
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

        // Print the info we just as yarp debug messages
        for (size_t j = 0; j < numAxes; ++j)
        {
            yInfo("Joint %s: enc1Res[%zu] = %u  (inv %.9f),  enc2Res[%zu] = %u  (inv %.9f)",
                  kClassName.data(),
                  j,
                  enc1Res[j],
                  enc1ResInv[j],
                  j,
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
                yDebug("Joint %s: axis %zu: 0x60A9=0x%08X → unit %s (1 unit = %.6f deg/s)",
                       kClassName.data(),
                       j,
                       raw,
                       name,
                       toDegS);
            else
                yWarning("Joint %s: axis %zu: failed to read 0x60A9, assuming 1 RPM (1 unit = 6 "
                         "deg/s)",
                         kClassName.data(),
                         j);
        }
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
                yWarning("Joint %s: gear‑ratio numerator not available (0x6091:01) → assume 1",
                         kClassName.data());
                num = 1U;
            }

            // ---- denominator ----------------------------------------------------
            if (ethercatManager.readSDO<uint32_t>(slaveIdx, 0x6091, 0x02, den)
                    != ::CiA402::EthercatManager::Error::NoError
                || den == 0U)
            {
                yWarning("Joint %s: gear‑ratio denominator not available/zero (0x6091:02) → assume "
                         "1",
                         kClassName.data());
                den = 1U;
            }

            yInfo("Joint %s: gear‑ratio %zu → %u : %u", kClassName.data(), j, num, den);

            // ---- cache value ----------------------------------------------------
            gearRatio[j] = static_cast<double>(num) / static_cast<double>(den);
        }

        // Pre‑compute the inverse for fast use elsewhere -------------------------
        for (size_t j = 0; j < numAxes; ++j)
        {
            gearRatioInv[j] = (gearRatio[j] != 0.0) ? (1.0 / gearRatio[j]) : 0.0;
        }

        return true;
    }

    bool readTorqueValues()
    {
        ratedMotorTorqueNm.resize(numAxes);
        maxMotorTorqueNm.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int s = firstSlave + int(j);
            uint32_t rated_mNm = 0;
            if (ethercatManager.readSDO<uint32_t>(s, 0x6076, 0x00, rated_mNm)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yError("%s: cannot read rated torque (0x6076:00)", kClassName.data());
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
                yError("%s: cannot read max torque (0x6072:00)", kClassName.data());
                return false;
            }
            maxMotorTorqueNm[j] = (double(maxPerm) / 1000.0) * ratedMotorTorqueNm[j];
            yInfo("Joint %s: MOTOR rated %.3f Nm, max %.3f Nm",
                  kClassName.data(),
                  ratedMotorTorqueNm[j],
                  maxMotorTorqueNm[j]);
        }

        return true;
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
                    rx->TargetTorque = tq_thousand;
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

                    switch (velLoopSrc[j])
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
                    rx->TargetVelocity = static_cast<int32_t>(std::llround(vel));
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
                    // CiA402 standard velocity - interpretation depends on drive's velocity loop
                    // source
                    if (velLoopSrc[j] == Impl::SensorSrc::Enc1)
                        return {double(v606C) * k, enc1Mount[j]};
                    if (velLoopSrc[j] == Impl::SensorSrc::Enc2)
                        return {double(v606C) * k, enc2Mount[j]};

                    // Fallback: if loop source unknown, prefer enc1 if available
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
                this->variables.jointPositions[j] = jointDeg;
                this->variables.motorEncoders[j] = motorDeg;
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
                this->variables.jointVelocities[j] = jointDegS;
                this->variables.motorVelocities[j] = motorDegS;
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
            this->variables.jointTorques[j] = motorNm * this->gearRatio[j];

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
            this->variables.feedbackTime[j]
                = tx.has(CiA402::TxField::Timestamp20F0)
                      ? double(tx.get<uint32_t>(CiA402::TxField::Timestamp20F0, 0))
                            * MICROSECONDS_TO_SECONDS
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
    // ---------------------------------------------------------------------
    // 0. Parse YARP parameters
    // ---------------------------------------------------------------------
    if (!cfg.check("ifname") || !cfg.find("ifname").isString())
    {
        yError("%s: 'ifname' parameter is not a string", Impl::kClassName.data());
        return false;
    }
    if (!cfg.check("num_axes") || !cfg.find("num_axes").isInt32())
    {
        yError("%s: 'num_axes' parameter is not an integer", Impl::kClassName.data());
        return false;
    }

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
            yError("%s: missing key '%s'", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Value& v = cfg.find(key);
        if (!v.isList())
        {
            yError("%s: key '%s' is not a list", Impl::kClassName.data(), key);
            return false;
        }

        const yarp::os::Bottle* lst = v.asList();
        if (!lst)
        {
            yError("%s: internal error: list for key '%s' is null", Impl::kClassName.data(), key);
            return false;
        }

        const size_t expected = static_cast<size_t>(m_impl->numAxes);
        const size_t actual = static_cast<size_t>(lst->size());
        if (actual != expected)
        {
            yError("%s: list for key '%s' has incorrect size (%zu), expected (%zu)",
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
                yError("%s: element %d in list for key '%s' is not a string",
                       Impl::kClassName.data(),
                       i,
                       key);
                result.clear();
                return false;
            }

            const std::string val = elem.asString();
            if (std::find(acceptedKeys.begin(), acceptedKeys.end(), val) == acceptedKeys.end())
            {
                yError("%s: invalid value '%s' in list for key '%s'",
                       Impl::kClassName.data(),
                       val.c_str(),
                       key);
                result.clear();
                return false;
            }

            result.push_back(val);
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
    yInfo("%s: opening EtherCAT manager on %s", Impl::kClassName.data(), ifname.c_str());

    // ---------------------------------------------------------------------
    // 1. Initialise the EtherCAT network via the high‑level manager
    // ---------------------------------------------------------------------

    const auto ecErr = m_impl->ethercatManager.init(ifname);
    if (ecErr != ::CiA402::EthercatManager::Error::NoError)
    {
        yError("%s: EtherCAT init failed (%d)", Impl::kClassName.data(), static_cast<int>(ecErr));
        return false;
    }

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
            yError("%s: failed to read position loop source (joint %zu)",
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

    yInfo("%s: using %zu axes, slaves %d ... %d",
          Impl::kClassName.data(),
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
            yError("%s: axis %zu: invalid config: %s", Impl::kClassName.data(), j, what);
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
    // 2. Idle every drive (switch‑on disabled, no mode selected)
    // ---------------------------------------------------------------------
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        const int slave = m_impl->firstSlave + static_cast<int>(j);
        auto* rx = m_impl->ethercatManager.getRxPDO(slave);
        if (!rx)
        {
            yError("%s: invalid slave index %d", Impl::kClassName.data(), slave);
            return false;
        }
        rx->Controlword = 0x0000;
        rx->OpMode = 0;
    }

    // Send one frame so the outputs take effect
    m_impl->ethercatManager.sendReceive();

    // ---------------------------------------------------------------------
    // 3. Fetch static SDO parameters (encoder resolutions, gear ratios)
    // ---------------------------------------------------------------------
    if (!m_impl->readEncoderResolutions())
    {
        yError("%s: failed to read encoder resolutions", Impl::kClassName.data());
        return false;
    }
    if (!m_impl->readSiVelocityUnits())
    {
        yError("%s: failed to read velocity conversions", Impl::kClassName.data());
        return false;
    }
    if (!m_impl->readGearRatios())
    {
        yError("%s: failed to read gear ratios", Impl::kClassName.data());
        return false;
    }
    if (!m_impl->readTorqueValues())
    {
        yError("%s: failed to read torque values", Impl::kClassName.data());
        return false;
    }

    // ---------------------------------------------------------------------
    // 4. Create YARP‑level containers and CiA‑402 state machines
    // ---------------------------------------------------------------------

    // 10. Resize the containers
    m_impl->variables.resizeContainers(m_impl->numAxes);
    m_impl->setPoints.resize(m_impl->numAxes);
    m_impl->setPoints.reset();
    m_impl->controlModeState.resize(m_impl->numAxes, VOCAB_CM_IDLE);
    m_impl->sm.resize(m_impl->numAxes);
    m_impl->velLatched.assign(m_impl->numAxes, false);
    m_impl->trqLatched.assign(m_impl->numAxes, false);
    m_impl->torqueSeedNm.assign(m_impl->numAxes, 0.0);

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->sm[j] = std::make_unique<CiA402::StateMachine>();
        m_impl->sm[j]->reset();
    }

    m_impl->fillJointNames();

    yInfo("%s: opened %zu axes. Initialization complete.",
          Impl::kClassName.data(),
          m_impl->numAxes);

    // ---------------------------------------------------------------------
    // 5. Launch the device thread
    // ---------------------------------------------------------------------
    if (!this->start())
    {
        yError("%s: failed to start the thread", Impl::kClassName.data());
        return false;
    }

    return true;
}

//  close()  —  stop the thread & release the NIC
bool CiA402MotionControl::close()
{
    this->stop(); // PeriodicThread → graceful stop
    yInfo("%s: EtheCAT master closed", Impl::kClassName.data());
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
                m_impl->velLatched[j] = m_impl->trqLatched[j] = false;
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
        yError("%s: sendReceive() failed", Impl::kClassName.data());
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
                newActive = Impl::ciaOpToYarp(m_impl->sm[j]->getActiveOpMode());
            }

            // If IDLE or FAULT or inhibited → clear SPs and latches; force target to IDLE on HW
            // inhibit
            if (newActive == VOCAB_CM_IDLE || newActive == VOCAB_CM_HW_FAULT
                || newActive == VOCAB_CM_FORCE_IDLE)
            {
                m_impl->setPoints.reset(j);
                m_impl->velLatched[j] = m_impl->trqLatched[j] = false;
                if (hwInhibit)
                {
                    m_impl->controlModeState.target[j] = VOCAB_CM_IDLE;
                }
            }

            // Detect mode entry to (re)arm first-cycle latches
            if (m_impl->controlModeState.active[j] != newActive)
            {
                // entering a control mode: arm latches and clear "has SP" flags
                m_impl->velLatched[j] = m_impl->trqLatched[j] = false;
                m_impl->setPoints.reset(j);
            }

            m_impl->controlModeState.active[j] = newActive;
        }
    }
}

// -----------------------------------------------
// ----------------- IMotorEncoders --------------
// -----------------------------------------------

bool CiA402MotionControl::getNumberOfMotorEncoders(int* num)
{
    if (num == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *num = static_cast<int>(m_impl->numAxes);
    return true;
}

bool CiA402MotionControl::resetMotorEncoder(int m)
{
    yError("%s: resetMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::resetMotorEncoders()
{
    yError("%s: resetMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    yError("%s: setMotorEncoderCountsPerRevolution() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoder(int m, const double v)
{
    yError("%s: setMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setMotorEncoders(const double*)
{
    yError("%s: setMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    if (cpr == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), m);
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), m);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), m);
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), m);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (m >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), m);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *ax = static_cast<int>(m_impl->numAxes);
    return true;
}

bool CiA402MotionControl::resetEncoder(int j)
{
    yError("%s: resetEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::resetEncoders()
{
    yError("%s: resetEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setEncoder(int j, const double val)
{
    yError("%s: setEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::setEncoders(const double* vals)
{
    yError("%s: setEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool CiA402MotionControl::getEncoder(int j, double* v)
{
    if (v == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: joint %d out of range", Impl::kClassName.data(), axis);
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n);
        return false;
    }

    {
        std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
        for (int k = 0; k < n; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
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
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    if (Impl::yarpToCiaOp(mode) < 0)
    {
        yError("%s: control mode %d not supported", Impl::kClassName.data(), mode);
        return false;
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    m_impl->controlModeState.target[j] = mode;
    return true;
}

bool CiA402MotionControl::setControlModes(const int n, const int* joints, int* modes)
{
    if (modes == nullptr || joints == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n);
        return false;
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    for (int k = 0; k < n; ++k)
    {
        if (joints[k] >= static_cast<int>(m_impl->numAxes))
        {
            yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
            return false;
        }
        m_impl->controlModeState.target[joints[k]] = modes[k];
    }
    return true;
}

bool CiA402MotionControl::setControlModes(int* modes)
{
    if (modes == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }

    std::lock_guard<std::mutex> l(m_impl->controlModeState.mutex);
    std::memcpy(m_impl->controlModeState.target.data(), modes, m_impl->numAxes * sizeof(int));
    return true;
}

//// -------------------------- ITorqueControl ------------------------------------
bool CiA402MotionControl::getTorque(int j, double* t)
{
    if (t == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // (a/b) Only accept if TORQUE is ACTIVE; otherwise reject (not considered)
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[j] != VOCAB_CM_TORQUE)
        {
            yError("%s: setRefTorque rejected: TORQUE mode is not active for the joint %d",
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        // check that all the joints are in TORQUE mode use std function without for loop
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] != VOCAB_CM_TORQUE)
            {
                yError("%s: setRefTorques rejected: TORQUE mode is not active for the joint "
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // check that all the joints are in TORQUE mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_TORQUE)
            {
                yError("%s: setRefTorques rejected: TORQUE mode is not active for the joint %d",
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    // (a/b) Only accept if VELOCITY is ACTIVE; otherwise reject
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);
        if (m_impl->controlModeState.active[j] != VOCAB_CM_VELOCITY)
        {
            yError("%s: velocityMove rejected: VELOCITY mode is not active for the joint %d",
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }

    // check that all the joints are in VELOCITY mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            if (m_impl->controlModeState.active[j] != VOCAB_CM_VELOCITY)
            {
                yError("%s: velocityMove rejected: VELOCITY mode is not active for the joint "
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (joint >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), joint);
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
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            vels[k] = m_impl->setPoints.jointVelocities[joints[k]];
        }
    }
    return true;
}

bool CiA402MotionControl::setRefAcceleration(int j, double acc)
{
    // no operation
    return false;
}

bool CiA402MotionControl::setRefAccelerations(const double* accs)
{
    // no operation
    return false;
}

bool CiA402MotionControl::getRefAcceleration(int j, double* acc)
{
    if (acc == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    *acc = 0.0; // CiA-402 does not support acceleration setpoints, so we return 0.0
    return true;
}

bool CiA402MotionControl::getRefAccelerations(double* accs)
{
    if (accs == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    std::memset(accs, 0, m_impl->numAxes * sizeof(double)); // CiA-402 does not support
                                                            // acceleration setpoints, so we
                                                            // return 0.0 for all axes
    return true;
}

bool CiA402MotionControl::stop(int j)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        m_impl->setPoints.jointVelocities[j] = 0.0;
        m_impl->setPoints.hasVelSP[j] = true;
    }
    return true;
}

bool CiA402MotionControl::stop()
{
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // check that all the joints are in VELOCITY mode
    {
        std::lock_guard<std::mutex> lock(m_impl->controlModeState.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
                return false;
            }
            if (m_impl->controlModeState.active[joints[k]] != VOCAB_CM_VELOCITY)
            {
                yError("%s: velocityMove rejected: VELOCITY mode is not active for the joint "
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    // no operation
    return true;
}

bool CiA402MotionControl::getRefAccelerations(const int n_joint, const int* joints, double* accs)
{
    if (accs == nullptr || joints == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
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
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    if (n_joint <= 0)
    {
        yError("%s: invalid number of joints %d", Impl::kClassName.data(), n_joint);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        for (int k = 0; k < n_joint; ++k)
        {
            if (joints[k] >= static_cast<int>(m_impl->numAxes))
            {
                yError("%s: joint %d out of range", Impl::kClassName.data(), joints[k]);
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
        yError("%s: getLastJointFault: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }

    const int slave = m_impl->firstSlave + j;

    // --- 0x603F:00 Error code (UINT16) ---
    uint16_t code = 0;
    auto err = m_impl->ethercatManager.readSDO<uint16_t>(slave, 0x603F, 0x00, code);
    if (err != ::CiA402::EthercatManager::Error::NoError)
    {
        yError("%s: getLastJointFault: SDO read 0x603F:00 failed (joint %d)",
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
} // namespace yarp::dev

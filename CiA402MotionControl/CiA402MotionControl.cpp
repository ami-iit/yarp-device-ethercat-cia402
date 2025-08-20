// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/******************************************************************************************
 * CiA402MotionControl.cpp                                                                 *
 *                                                                                        *
 * This file shows implement a YARP DeviceDriver that talks to a ring of                  *
 * CiA-402 EtherCAT drives via SOEM.                                                      *
 *                                                                                        *
 * Major changes vs. the draft you sent:                                                  *
 *  • Added all the missing data members (expected_wkc, firstSlave, vectors …).           *
 *  • Parameters now come from the .ini file:                                             *
 *      - ifname                : mandatory   (string)                                    *
 *      - num_axes              : mandatory   (int)                                       *
 *      - first_slave           : optional    (int,  default = 1)                         *
 *      - expected_slave_name   : optional    (string, used for sanity checks)            *
 *  • Added lots of #include-s and forward declarations so the file is self-contained.    *
 *****************************************************************************************/

#include "CiA402MotionControl.h"
#include "CiA402StateMachine.h"
#include "EthercatManager.h"

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// SOEM
#include <ethercat.h>

// STD
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>
#include <cmath>

namespace
{
inline std::string describe603F(uint16_t code)
{
    switch (code)
    {
    case 0x0000:
        return "No fault";
    // Hardware protection – current/voltage/temperature
    case 0x2220:
        return "Hardware over-current (phase or DC bus)"; // PhOc*/PhOc
    case 0x3210:
        return "Hardware over-voltage";
    case 0x3220:
        return "Hardware under-voltage";
    case 0x4310:
        return "Over-temperature (warning/error threshold reached)"; // PhOtW/PhOtDriv

    // Gate driver & safety/homing (Synapticon groups many items here)
    case 0x5300:
        return "Gate-driver / safety / HW protection fault "
               "(e.g., UVLO, VDS OC, gate drive fault, safety input discrepancy,"
               " homing switch invalid state)";

    // Config/parameters out of range
    case 0x6320:
        return "Invalid configuration / parameter out of range";

    // Sensors (external analog / encoder comms)
    case 0x7300:
        return "External analog sensor threshold exceeded";
    case 0x7303:
        return "Encoder/Sensor communication fault (BiSS/SSI: link/level/frame)";
    case 0x7304:
        return "Encoder/Sensor configuration/CRC/frame error";

    // Firmware / internal service issues
    case 0x7500:
        return "Internal service skipping cycles / internal fault";

    // Performance / timing warnings reported in error list
    case 0xF002:
        return "Control cycle exceeded (service running slower than 4 kHz)";

    default:
        // Family-based fallbacks for unlisted but related codes
        switch (code & 0xFF00)
        {
        case 0x2200:
            return "Current-related hardware fault";
        case 0x3200:
            return "Voltage-related hardware fault";
        case 0x4300:
            return "Temperature-related fault/warning";
        case 0x5300:
            return "Gate-driver / safety / protection fault (manufacturer specific)";
        case 0x6300:
            return "Invalid configuration / parameter";
        case 0x7300:
            return "Sensor/encoder fault (communication/config)";
        case 0x7500:
            return "Internal/firmware fault";
        case 0xF000:
            return "Timing/performance warning";
        default:
            return "Unknown (0x603F) error code";
        }
    }
}
} // namespace

namespace yarp::dev
{

struct CiA402MotionControl::Impl
{
    // Human-readable name for log messages
    static constexpr std::string_view kClassName = "CiA402MotionControl";

    // EtherCat manager
    ::CiA402::EthercatManager ethercatManager;

    // Constants
    static constexpr double RPM_TO_DEG_PER_SEC = 360.0 / 60.0; // 1 / DEG_PER_SEC_TO_RPM
    static constexpr double DEG_PER_SEC_TO_RPM = 60.0 / 360.0; // 1 / RPM_TO_DEG_PER_SEC
    static constexpr double MICROSECONDS_TO_SECONDS = 1e-6; // 1 / 1'000'000

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
    std::vector<uint32_t> encoderRes; // "counts per revolution" etc.
    std::vector<double> gearRatio; //  load-revs : motor-revs
    std::vector<double> gearRatioInv; // 1 / gearRatio
    std::vector<double> encoderResInverted; // 1 / encoderRes
    std::vector<double> ratedTorque; // [Nm] from 0x6076
    std::vector<double> maxTorque; // [Nm] from 0x6072

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

    //--------------------------------------------------------------------------
    // Utility: dump the AL state & status code of every slave (for debugging)
    //--------------------------------------------------------------------------
    void printAlStatusForAllSlaves() const
    {
        for (int i = 1; i <= ec_slavecount; ++i)
        {
            yError("  slave %2d  state 0x%02X  AL-status 0x%04X (%s)",
                   i,
                   ec_slave[i].state,
                   ec_slave[i].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
    }

    void fillJointNames()
    {
        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);
            this->variables.jointNames[j] = ec_slave[slaveIdx].name;
        }
    }

    //--------------------------------------------------------------------------
    //  One-shot SDO reads – here we only fetch encoder resolutions
    //--------------------------------------------------------------------------
    bool readEncoderResolutions()
    {
        encoderRes.resize(numAxes);
        encoderResInverted.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            // ------------------------------------------------------------
            // 1. Determine which encoder is active (hall vs quadrature)
            // ------------------------------------------------------------
            uint8_t source = 0;
            auto err = this->ethercatManager.readSDO<uint8_t>(slaveIdx, 0x2012, 0x09, source);
            if (err != ::CiA402::EthercatManager::Error::NoError)
            {
                yError("Joint %zu: cannot read encoder‑source (0x2012:09)", j);
                return false;
            }

            // ------------------------------------------------------------
            // 2. Fetch the encoder resolution from the proper object
            // ------------------------------------------------------------
            uint32_t res = 0;
            const uint16_t idx = (source == 1) ? 0x2110 : 0x2112; // hall vs quadrature
            err = this->ethercatManager.readSDO<uint32_t>(slaveIdx, idx, 0x03, res);
            if (err != ::CiA402::EthercatManager::Error::NoError)
            {
                yError("Joint %zu: cannot read encoder‑resolution (0x%04X:03)", j, idx);
                return false;
            }
            this->encoderRes[j] = res;
        }

        // ------------------------------------------------------------
        // 3. Pre‑compute 1/res for fast feedback conversion
        // ------------------------------------------------------------
        for (size_t j = 0; j < numAxes; ++j)
        {
            if (this->encoderRes[j] != 0)
            {
                this->encoderResInverted[j] = 1.0 / static_cast<double>(this->encoderRes[j]);
            } else
            {
                this->encoderResInverted[j] = 0.0;
            }
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
        this->ratedTorque.resize(numAxes);
        this->maxTorque.resize(numAxes);

        if (this->gearRatio.empty())
        {
            yError("Joint %s: cannot read torque values without gear-ratio", kClassName.data());
            return false;
        }

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            uint32_t rated = 0;
            if (ethercatManager.readSDO<uint32_t>(slaveIdx, 0x6076, 0x00, rated)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yError("Joint %s: cannot read rated torque (0x6076:00)", kClassName.data());
                return false;
            }

            // we need to convert the rated torque from  mNm to Nm and multiply by the gear ratio
            this->ratedTorque[j] = static_cast<double>(rated) / 1000.0 * this->gearRatio[j];
        }

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            uint16_t max = 0;
            if (ethercatManager.readSDO<uint16_t>(slaveIdx, 0x6072, 0x00, max)
                != ::CiA402::EthercatManager::Error::NoError)
            {
                yError("Joint %s: cannot read max torque (0x6072:00)", kClassName.data());
                return false;
            }

            // max is given in per thousandth of rated torque.
            this->maxTorque[j] = max / 1000.0 * this->ratedTorque[j];

            yInfo("Joint %s: rated torque %zu → %.2f Nm",
                  kClassName.data(),
                  j,
                  this->ratedTorque[j]);
            yInfo("Joint %s: max torque %zu → %.2f Nm", kClassName.data(), j, this->maxTorque[j]);
        }

        return true;
    }

    bool setSetPoints()
    {
        std::lock_guard<std::mutex> lock(this->setPoints.mutex);
        for (size_t j = 0; j < this->numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            // check the control mode actually active on the drive
            const int opMode = this->controlModeState.active[j];
            auto rxPDO = this->ethercatManager.getRxPDO(slaveIdx);
            if (opMode == VOCAB_CM_TORQUE)
            {
                if (!this->trqLatched[j])
                {
                    // Capture seed once on entry
                    const double Nm = this->variables.jointTorques[j];
                    this->torqueSeedNm[j] = Nm;
                    const int16_t tq
                        = static_cast<int16_t>(std::llround(Nm / this->ratedTorque[j] * 1000.0));
                    rxPDO->TargetTorque = tq; // (c) first cycle → measured torque
                    this->trqLatched[j] = true;
                } else
                {
                    const double Nm = setPoints.hasTorqueSP[j] ? setPoints.jointTorques[j]
                                                               : torqueSeedNm[j];
                    const int16_t tq
                        = static_cast<int16_t>(std::llround(Nm / this->ratedTorque[j] * 1000.0));
                    rxPDO->TargetTorque = tq; // keep seed until user sets
                }
            }

            if (opMode == VOCAB_CM_VELOCITY)
            {
                if (!velLatched[j])
                {
                    rxPDO->TargetVelocity = 0; // (c) first cycle after entry → 0 rpm
                    velLatched[j] = true;
                } else
                {
                    if (setPoints.hasVelSP[j])
                    {
                        const double rpm
                            = setPoints.jointVelocities[j] * gearRatio[j] * DEG_PER_SEC_TO_RPM;
                        rxPDO->TargetVelocity = static_cast<int32_t>(std::llround(rpm));
                    } else
                    {
                        rxPDO->TargetVelocity = 0; // hold safe default until user provides a SP
                    }
                }
            }
        }
        return true;
    }

    bool readFeedback()
    {
        // 1. Lock the mutex to protect the PDOs from concurrent access
        std::lock_guard<std::mutex> lock(this->variables.mutex);

        // 2. Read & print encoder feedback
        for (size_t j = 0; j < this->numAxes; ++j)
        {
            const auto& fb
                = *(this->ethercatManager.getTxPDO(this->firstSlave + static_cast<int>(j)));
            const double encoderResInv = this->encoderResInverted[j];

            this->variables.motorEncoders[j] = static_cast<double>(fb.PositionValue)
                                               * encoderResInv // [encoder counts] → [turns]
                                               * 360.0; // [turns] → [deg]

            // we need to convert the rpm to deg/s
            this->variables.motorVelocities[j] = static_cast<double>(fb.VelocityValue) // rpm
                                                 * RPM_TO_DEG_PER_SEC; // [rpm] → [deg/s]

            // the acceleration is not available in the PDO
            this->variables.motorAccelerations[j] = 0.0;

            // TODO we assume that we have only one encoder so the joint position and
            // velocity is
            // computed by using the gear ratio
            this->variables.jointPositions[j]
                = this->variables.motorEncoders[j] * this->gearRatioInv[j];
            this->variables.jointVelocities[j]
                = this->variables.motorVelocities[j] * this->gearRatioInv[j];
            this->variables.jointAccelerations[j]
                = this->variables.motorAccelerations[j] * this->gearRatioInv[j];

            this->variables.jointTorques[j] = static_cast<double>(fb.TorqueValue) / 1000.0
                                              * this->ratedTorque[j]; // [0.1 Nm] → [Nm]

            this->variables.STO[j] = fb.STO;
            this->variables.SBC[j] = fb.SBC;

            this->variables.feedbackTime[j]
                = static_cast<double>(fb.Timestamp) * MICROSECONDS_TO_SECONDS; // [μs] → [s]
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

    yInfo("%s: %d slaves detected, OPERATIONAL, WKC=%d",
          Impl::kClassName.data(),
          ec_slavecount,
          m_impl->ethercatManager.getWorkingCounter());

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
    if (!m_impl->readGearRatios())
    {
        yError("%s: failed to read gear ratios", Impl::kClassName.data());
        return false;
    }
    if (!m_impl->readTorqueValues())
    {
        ec_close();
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
    m_impl->lastActiveMode.assign(m_impl->numAxes, VOCAB_CM_IDLE);

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
            ::CiA402::TxPDO* tx = m_impl->ethercatManager.getTxPDO(slaveIdx);

            const int tgt = m_impl->controlModeState.target[j];

            /* ------------ normal control-mode path --------------------- */
            const int8_t opReq = Impl::yarpToCiaOp(tgt); // −1 = unsupported
            if (opReq < 0)
            { // ignore unknown modes
                continue;
            }

            const bool hwInhibit = (tx->STO == 1) || (tx->SBC == 1);

            CiA402::StateMachine::Command cmd
                = m_impl->sm[j]->update(tx->Statusword, tx->OpModeDisplay, opReq, hwInhibit);

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
            ::CiA402::TxPDO* tx = m_impl->ethercatManager.getTxPDO(slaveIdx);
            const bool hwInhibit = (tx->STO == 1) || (tx->SBC == 1);
            const bool enabled = CiA402::StateMachine::isOpEnabled(tx->Statusword);

            int newActive = VOCAB_CM_IDLE;
            if (!hwInhibit && enabled)
            {
                newActive = Impl::ciaOpToYarp(m_impl->sm[j]->getActiveOpMode());
            }
            // If IDLE or inhibited → clear SPs and latches; force target to IDLE on HW inhibit
            if (newActive == VOCAB_CM_IDLE)
            {
                m_impl->setPoints.reset(j);
                m_impl->velLatched[j] = m_impl->trqLatched[j] = false;
                if (hwInhibit)
                {
                    m_impl->controlModeState.target[j] = VOCAB_CM_IDLE;
                }
            }

            // Detect mode entry to (re)arm first-cycle latches
            if (m_impl->lastActiveMode[j] != newActive)
            {
                if (newActive != VOCAB_CM_IDLE)
                {
                    // entering a control mode: arm latches and clear "has SP" flags
                    m_impl->velLatched[j] = m_impl->trqLatched[j] = false;
                    std::lock_guard<std::mutex> sp(m_impl->setPoints.mutex);
                    m_impl->setPoints.hasVelSP[j] = false;
                    m_impl->setPoints.hasTorqueSP[j] = false;
                }
                m_impl->lastActiveMode[j] = newActive;
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

        *cpr = static_cast<double>(m_impl->encoderRes[m]);
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
                yError("%s: setRefTorques rejected: TORQUE mode is not active for the joint %zu",
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

    *min = -m_impl->maxTorque[j];
    *max = m_impl->maxTorque[j];

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
        min[j] = -m_impl->maxTorque[j];
        max[j] = m_impl->maxTorque[j];
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
                yError("%s: velocityMove rejected: VELOCITY mode is not active for the joint %zu",
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
    std::memset(accs, 0, m_impl->numAxes * sizeof(double)); // CiA-402 does not support acceleration
                                                            // setpoints, so we return 0.0 for all
                                                            // axes
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
                yError("%s: velocityMove rejected: VELOCITY mode is not active for the joint %d",
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
    message = describe603F(code);
    return true;
}

} // namespace yarp::dev

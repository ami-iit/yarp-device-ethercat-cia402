// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/******************************************************************************************
 * Ds402MotionControl.cpp                                                                 *
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

#include "Ds402MotionControl.h"
#include "Cia402StateMachine.h"
#include "EthercatManager.h"

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// SOEM
#include <ethercat.h>

// STD
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace yarp::dev
{

struct Ds402MotionControl::Impl
{
    // Human-readable name for log messages
    static constexpr std::string_view kClassName = "Ds402MotionControl";

    // EtherCat manager
    ::Cia402::EthercatManager ethercatManager;

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

        void resize(std::size_t n)
        {
            jointTorques.resize(n);
        }

        void reset()
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            std::fill(this->jointTorques.begin(), this->jointTorques.end(), 0.0);
        }
    };

    SetPoints setPoints;

    /* --------------- CiA-402 power-state machine ------------------------ */
    std::vector<std::unique_ptr<Cia402::StateMachine>> sm;

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
            if (err != ::Cia402::EthercatManager::Error::NoError)
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
            if (err != ::Cia402::EthercatManager::Error::NoError)
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
                != ::Cia402::EthercatManager::Error::NoError)
            {
                yWarning("Joint %s: gear‑ratio numerator not available (0x6091:01) → assume 1",
                         kClassName.data());
                num = 1U;
            }

            // ---- denominator ----------------------------------------------------
            if (ethercatManager.readSDO<uint32_t>(slaveIdx, 0x6091, 0x02, den)
                    != ::Cia402::EthercatManager::Error::NoError
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
                != ::Cia402::EthercatManager::Error::NoError)
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
                != ::Cia402::EthercatManager::Error::NoError)
            {
                yError("Joint %s: cannot read max torque (0x6072:00)", kClassName.data());
                return false;
            }

            // max is given in per thousandth of rated torque.
            this->maxTorque[j] = max / 1000.0 * this->ratedTorque[j];


            yInfo("Joint %s: rated torque %zu → %.2f Nm", kClassName.data(), j,
                   this->ratedTorque[j]);
            yInfo("Joint %s: max torque %zu → %.2f Nm", kClassName.data(), j,
                   this->maxTorque[j]);
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
                // the torque is the thousand of the rated torque.
                int16_t torque = static_cast<int16_t>(this->setPoints.jointTorques[j]
                                                      / this->ratedTorque[j] * 1000.0);
                rxPDO->TargetTorque = torque;
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

//  Ds402MotionControl  —  ctor / dtor

Ds402MotionControl::Ds402MotionControl(double period, yarp::os::ShouldUseSystemClock useSysClock)
    : yarp::os::PeriodicThread(period, useSysClock, yarp::os::PeriodicThreadClock::Absolute)
    , m_impl(std::make_unique<Impl>())
{
}

Ds402MotionControl::Ds402MotionControl()
    : yarp::os::PeriodicThread(0.001 /*1 kHz*/,
                               yarp::os::ShouldUseSystemClock::Yes,
                               yarp::os::PeriodicThreadClock::Absolute)
    , m_impl(std::make_unique<Impl>())
{
}

Ds402MotionControl::~Ds402MotionControl() = default;

//  open()  —  bring the ring to OPERATIONAL and start the cyclic thread

bool Ds402MotionControl::open(yarp::os::Searchable& cfg)
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
    if (ecErr != ::Cia402::EthercatManager::Error::NoError)
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

    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        m_impl->sm[j] = std::make_unique<Cia402::StateMachine>();
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
bool Ds402MotionControl::close()
{
    this->stop(); // PeriodicThread → graceful stop
    yInfo("%s: EtheCAT master closed", Impl::kClassName.data());
    return true;
}

//  run()  —  gets called every period (real-time control loop)
void Ds402MotionControl::run()
{
    /* ------------------------------------------------------------------
     * 1.  APPLY USER-REQUESTED CONTROL MODES (CiA-402 power machine)
     * ----------------------------------------------------------------*/
    {
        std::lock_guard<std::mutex> g(m_impl->controlModeState.mutex);

        for (size_t j = 0; j < m_impl->numAxes; ++j)
        {
            const int slaveIdx = m_impl->firstSlave + static_cast<int>(j);
            ::Cia402::RxPDO* rx = m_impl->ethercatManager.getRxPDO(slaveIdx);
            ::Cia402::TxPDO* tx = m_impl->ethercatManager.getTxPDO(slaveIdx);

            const int tgt = m_impl->controlModeState.target[j];

            // /* ------------ FORCE-IDLE  (fault-reset + idle) ------------- */
            // if (tgt == VOCAB_CM_FORCE_IDLE)
            // {
            //     Cia402::StateMachine::Command cmd = m_impl->sm[j]->faultReset(); // 0x0080 +
            //     OpMode
            //         // 0
            //         rx->Controlword
            //         = cmd.controlword;
            //     if (cmd.writeOpMode)
            //     {
            //         rx->OpMode = cmd.opMode;
            //     }
            //     continue;
            // }

            /* ------------ normal control-mode path --------------------- */
            const int8_t opReq = Impl::yarpToCiaOp(tgt); // −1 = unsupported
            if (opReq < 0)
            { // ignore unknown modes
                continue;
            }

            Cia402::StateMachine::Command cmd
                = m_impl->sm[j]->update(tx->Statusword, tx->OpModeDisplay, opReq);

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
    if (m_impl->ethercatManager.sendReceive() != ::Cia402::EthercatManager::Error::NoError)
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
            ::Cia402::TxPDO* tx = m_impl->ethercatManager.getTxPDO(slaveIdx);
            const uint8_t SBC = tx->SBC;
            const uint8_t STO = tx->STO;

            if (SBC == 1 || STO == 1)
            {
                m_impl->controlModeState.active[j] = VOCAB_CM_IDLE;
                m_impl->controlModeState.target[j] = VOCAB_CM_IDLE;

                // we reset the setpoints to 0.0
                m_impl->setPoints.reset();

                continue;
            }

            m_impl->controlModeState.active[j]
                = Impl::ciaOpToYarp(m_impl->sm[j]->getActiveOpMode());
        }
    }
}

// -----------------------------------------------
// ----------------- IMotorEncoders --------------
// -----------------------------------------------

bool Ds402MotionControl::getNumberOfMotorEncoders(int* num)
{
    if (num == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *num = static_cast<int>(m_impl->numAxes);
    return true;
}

bool Ds402MotionControl::resetMotorEncoder(int m)
{
    yError("%s: resetMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::resetMotorEncoders()
{
    yError("%s: resetMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    yError("%s: setMotorEncoderCountsPerRevolution() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::getMotorEncoderCountsPerRevolution(int m, double* cpr)
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

bool Ds402MotionControl::setMotorEncoder(int m, const double val)
{
    yError("%s: setMotorEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::setMotorEncoders(const double* vals)
{
    yError("%s: setMotorEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::getMotorEncoder(int m, double* v)
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

bool Ds402MotionControl::getMotorEncoders(double* encs)
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

bool Ds402MotionControl::getMotorEncodersTimed(double* encs, double* time)
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

bool Ds402MotionControl::getMotorEncoderTimed(int m, double* encs, double* time)
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

bool Ds402MotionControl::getMotorEncoderSpeed(int m, double* sp)
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

bool Ds402MotionControl::getMotorEncoderSpeeds(double* spds)
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

bool Ds402MotionControl::getMotorEncoderAcceleration(int m, double* acc)
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

bool Ds402MotionControl::getMotorEncoderAccelerations(double* accs)
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
bool Ds402MotionControl::getEncodersTimed(double* encs, double* time)
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

bool Ds402MotionControl::getEncoderTimed(int j, double* encs, double* time)
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

bool Ds402MotionControl::getAxes(int* ax)
{
    if (ax == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    *ax = static_cast<int>(m_impl->numAxes);
    return true;
}

bool Ds402MotionControl::resetEncoder(int j)
{
    yError("%s: resetEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::resetEncoders()
{
    yError("%s: resetEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::setEncoder(int j, const double val)
{
    yError("%s: setEncoder() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::setEncoders(const double* vals)
{
    yError("%s: setEncoders() not implemented", Impl::kClassName.data());
    return false;
}

bool Ds402MotionControl::getEncoder(int j, double* v)
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

bool Ds402MotionControl::getEncoders(double* encs)
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

bool Ds402MotionControl::getEncoderSpeed(int j, double* sp)
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

bool Ds402MotionControl::getEncoderSpeeds(double* spds)
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

bool Ds402MotionControl::getEncoderAcceleration(int j, double* spds)
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

bool Ds402MotionControl::getEncoderAccelerations(double* accs)
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

bool Ds402MotionControl::getAxisName(int j, std::string& name)
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

bool Ds402MotionControl::getJointType(int axis, yarp::dev::JointTypeEnum& type)
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
bool Ds402MotionControl::getControlMode(int j, int* mode)
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

bool Ds402MotionControl::getControlModes(int* modes)
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

bool Ds402MotionControl::getControlModes(const int n, const int* joints, int* modes)
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

bool Ds402MotionControl::setControlMode(const int j, const int mode)
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

bool Ds402MotionControl::setControlModes(const int n, const int* joints, int* modes)
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

bool Ds402MotionControl::setControlModes(int* modes)
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
bool Ds402MotionControl::getTorque(int j, double* t)
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

bool Ds402MotionControl::getTorques(double* t)
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

bool Ds402MotionControl::getRefTorque(int j, double* t)
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
        *t = m_impl->setPoints.jointTorques[j];
    }
    return true;
}

bool Ds402MotionControl::getRefTorques(double* t)
{
    if (t == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->variables.mutex);
        std::memcpy(t, m_impl->setPoints.jointTorques.data(), m_impl->numAxes * sizeof(double));
    }
    return true;
}

bool Ds402MotionControl::setRefTorque(int j, double t)
{
    if (j >= static_cast<int>(m_impl->numAxes))
    {
        yError("%s: joint %d out of range", Impl::kClassName.data(), j);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        m_impl->setPoints.jointTorques[j] = t;
    }
    return false;
}

bool Ds402MotionControl::setRefTorques(const double* t)
{
    if (t == nullptr)
    {
        yError("%s: null pointer", Impl::kClassName.data());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_impl->setPoints.mutex);
        std::memcpy(m_impl->setPoints.jointTorques.data(), t, m_impl->numAxes * sizeof(double));
    }
    return false;
}

bool Ds402MotionControl::getTorqueRange(int j, double* min, double* max)
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

bool Ds402MotionControl::getTorqueRanges(double* min, double* max)
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

} // namespace yarp::dev

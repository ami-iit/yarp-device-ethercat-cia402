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

// Forward declarations for the process-data structs that each drive exposes.
// TODO: move to a separate header file better if we can set if from configuratrion file
#pragma pack(push, 1)
struct FeedbackPDO // (= slave → master, also called *Tx*PDO)
{
    uint16_t Statusword;
    int8_t OpModeDisplay;
    int32_t PositionValue; // [encoder counts]
    int32_t VelocityValue; // [rpm]
    int16_t TorqueValue; // [0.1 Nm]
    int32_t PositionErrorActualValue;
    uint16_t AnalogInput1;
    uint16_t AnalogInput2;
    uint16_t AnalogInput3;
    uint16_t AnalogInput4;
    uint32_t TuningStatus;
    uint32_t DigitalInputs;
    uint32_t UserMISO;
    uint32_t Timestamp;
    int32_t PositionDemandInternalValue;
    int32_t VelocityDemandValue;
    int16_t TorqueDemand;
};

struct CommandPDO // (= master → slave, also called *Rx*PDO)
{
    uint16_t Controlword;
    int8_t OpMode;
    int16_t TargetTorque;
    int32_t TargetPosition;
    int32_t TargetVelocity;
    int16_t TorqueOffset;
    int32_t TuningCommand;
    int32_t PhysicalOutputs;
    int32_t BitMask;
    int32_t UserMOSI;
    int32_t VelocityOffset;
};
#pragma pack(pop)

namespace yarp::dev
{

// Forward declarations expected by template code
using RxPDO = CommandPDO; // outputs (master → slave)
using TxPDO = FeedbackPDO; // inputs  (slave → master)

struct Ds402MotionControl::Impl
{
    // Human-readable name for log messages
    static constexpr std::string_view kClassName = "Ds402MotionControl";

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
    uint32_t expectedWkc{0}; // for fast I/O integrity check
    std::vector<RxPDO*> rx; // one ptr per joint → command area
    std::vector<TxPDO*> tx; // one ptr per joint → feedback area
    std::vector<uint32_t> encoderRes; // "counts per revolution" etc.
    std::vector<double> gearRatio; //  load-revs : motor-revs
    std::vector<double> gearRatioInv; // 1 / gearRatio
    std::vector<double> encoderResInverted; // 1 / encoderRes

    struct VariablesReadFromMotors
    {
        std::mutex mutex; // protect the variables in this structure from concurrent access
        std::vector<double> motorEncoders; // for getMotorEncoders()
        std::vector<double> motorVelocities; // for getMotorVelocities()
        std::vector<double> motorAccelerations; // for getMotorAccelerations()
        std::vector<double> jointPositions; // for getJointPositions()
        std::vector<double> jointVelocities; // for getJointVelocities()
        std::vector<double> jointAccelerations; // for getJointAccelerations()
        std::vector<double> feedbackTime; // feedback time in seconds
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
        }
    };

    VariablesReadFromMotors variables;

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
    //  Build rx[] / tx[] vectors so that joint j → slave (firstSlave + j)
    //--------------------------------------------------------------------------
    bool buildJointIoPointers()
    {
        this->rx.resize(numAxes);
        this->tx.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            // Sanity-check: is the bus long enough?
            if (slaveIdx > ec_slavecount)
            {
                yError("%s: joint %zu → slave %d (out of range)", kClassName.data(), j, slaveIdx);
                return false;
            }

            // Optional: verify the name if the user gave one
            if (!expectedName.empty()
                && std::strcmp(ec_slave[slaveIdx].name, expectedName.c_str()) != 0)
            {
                yError("%: joint %zu → slave %d (%s) does not match expected name '%s'",
                       kClassName.data(),
                       j,
                       slaveIdx,
                       ec_slave[slaveIdx].name,
                       expectedName.c_str());
                return false;
            }

            // Set the pointers to the PDO areas
            rx[j] = reinterpret_cast<RxPDO*>(ec_slave[slaveIdx].outputs);
            tx[j] = reinterpret_cast<TxPDO*>(ec_slave[slaveIdx].inputs);
            if (rx[j] == nullptr || tx[j] == nullptr)
            {
                yError("%s: joint %zu → slave %d has no PDOs", kClassName.data(), j, slaveIdx);
                return false;
            }
        }
        return true;
    }

    //--------------------------------------------------------------------------
    //  One-shot SDO reads – here we only fetch encoder resolutions
    //--------------------------------------------------------------------------
    bool readEncoderResolutions()
    {
        this->encoderRes.resize(numAxes);
        this->encoderResInverted.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            uint8_t source = 0;
            int sz = sizeof(source);
            if (ec_SDOread(slaveIdx, 0x2012, 0x09, false, &sz, &source, EC_TIMEOUTRXM) <= 0)
            {
                yError("Joint %s: cannot read encoder-source (0x2012:09)", kClassName.data());
                return false;
            }

            uint32_t res = 0;
            sz = sizeof(res);
            const uint16_t idx = (source == 1) ? 0x2110 : 0x2112; // hall vs. quadrature
            if (ec_SDOread(slaveIdx, idx, 0x03, false, &sz, &res, EC_TIMEOUTRXM) <= 0)
            {
                yError("Joint %s: cannot read encoder-resolution (0x%04X:03)",
                       kClassName.data(),
                       idx);
                return false;
            }
            this->encoderRes[j] = res;
        }

        // 1. Invert the encoder resolutions for later use in getFeedback()
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
        this->gearRatio.resize(numAxes);
        this->gearRatioInv.resize(numAxes);

        for (size_t j = 0; j < numAxes; ++j)
        {
            const int slaveIdx = firstSlave + static_cast<int>(j);

            uint32_t num = 1; // default numerator
            uint32_t den = 1; // default denominator
            int sz;

            /* ---- numerator -------------------------------------------------- */
            sz = sizeof(num);
            if (ec_SDOread(slaveIdx, 0x6091, 0x01, false, &sz, &num, EC_TIMEOUTRXM) <= 0)
            {
                yWarning("Joint %s: gear-ratio numerator not available (0x6091:01) → assume 1",
                         kClassName.data());
                num = 1;
            }

            /* ---- denominator ------------------------------------------------ */
            sz = sizeof(den);
            if (ec_SDOread(slaveIdx, 0x6091, 0x02, false, &sz, &den, EC_TIMEOUTRXM) <= 0
                || den == 0)
            {
                yWarning("Joint %s: gear-ratio denominator not available/zero (0x6091:02) → assume "
                         "1",
                         kClassName.data());
                den = 1;
            }

            yInfo("Joint %s: gear-ratio %zu → %u : %u", kClassName.data(), j, num, den);

            /* ---- cache values ---------------------------------------------- */
            this->gearRatio[j] = static_cast<double>(num) / static_cast<double>(den);
        }

        /* Pre-compute 1 / ratio for fast use elsewhere ----------------------- */
        for (size_t j = 0; j < numAxes; ++j)
        {
            if (this->gearRatio[j] != 0.0)
            {
                this->gearRatioInv[j] = 1.0 / this->gearRatio[j];
            } else
            {
                this->gearRatioInv[j] = 0.0;
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
            const auto& fb = *(this->tx[j]);
            const double encoderResInv = this->encoderResInverted[j];

            this->variables.motorEncoders[j] = static_cast<double>(fb.PositionValue)
                                               * encoderResInv // [encoder counts] → [turns]
                                               * 360.0; // [turns] → [deg]

            // we need to convert the rpm to deg/s
            this->variables.motorVelocities[j] = static_cast<double>(fb.VelocityValue) // rpm
                                                 * RPM_TO_DEG_PER_SEC; // [rpm] → [deg/s]

            // the acceleration is not available in the PDO
            this->variables.motorAccelerations[j] = 0.0;

            // TODO we assume that we have only one encoder so the joint position and velocity is
            // computed by using the gear ratio
            this->variables.jointPositions[j]
                = this->variables.motorEncoders[j] * this->gearRatioInv[j];
            this->variables.jointVelocities[j]
                = this->variables.motorVelocities[j] * this->gearRatioInv[j];
            this->variables.jointAccelerations[j]
                = this->variables.motorAccelerations[j] * this->gearRatioInv[j];

            this->variables.feedbackTime[j]
                = static_cast<double>(fb.Timestamp) * MICROSECONDS_TO_SECONDS; // [μs] → [s]
        }

        return true;
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

    //  0. Read parameters from the YARP xml file
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
    {
        m_impl->expectedName = cfg.find("expected_slave_name").asString();
    }

    const std::string ifname = cfg.find("ifname").asString();
    yInfo("%s: opening EtherCAT master on %s", Impl::kClassName.data(), ifname.c_str());

    //  1. ec_init()  — raw Ethernet socket + internal buffers
    if (ec_init(ifname.c_str()) <= 0)
    {
        yError("%s: ec_init() failed", Impl::kClassName.data());
        return false;
    }

    //  2. Scan the ring -> PRE-OP
    if (ec_config_init(false /*fresh scan*/) <= 0)
    {
        yError("%s: ec_config_init() failed", Impl::kClassName.data());
        ec_close();
        return false;
    }
    yInfo("%s: found %d slaves", Impl::kClassName.data(), ec_slavecount);

    //  3. Build process image + enable distributed clocks (if any)
    if (ec_config_map(m_impl->ioMap) <= 0)
    {
        yError("%s: ec_config_map() failed", Impl::kClassName.data());
        ec_close();
        return false;
    }
    ec_configdc();

    //  4. Wait until the whole ring reaches SAFE-OP (inputs valid)
    constexpr std::size_t allSlaves = 0; // 0 = all slaves
    ec_statecheck(allSlaves, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP)
    {
        yError("%s: ring failed to reach SAFE-OP", Impl::kClassName.data());
        m_impl->printAlStatusForAllSlaves();
        ec_close();
        return false;
    }

    //  5. Pre-compute expected WKC for sanity checks in run()
    m_impl->expectedWkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    yInfo("%s: expected WKC = %d", Impl::kClassName.data(), m_impl->expectedWkc);

    //  6. Ask for OPERATIONAL and wait (max 10 s)
    {
        for (std::size_t slave_id = 0; slave_id < ec_slavecount; ++slave_id)
        {
            ec_slave[slave_id].state = EC_STATE_OPERATIONAL;
        }

        // send one valid process data to make outputs in slaves happy
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        // request OP state for all slaves
        ec_writestate(0);

        size_t attempts = 200; // 200 × 50 ms = 10 s
        constexpr int timeout = 50'000; // 50 ms -> 50'000 μs
        do
        {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(allSlaves, EC_STATE_OPERATIONAL, timeout);
        } while (attempts-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    }

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        yError("%s: ring failed to reach OPERATIONAL", Impl::kClassName.data());
        m_impl->printAlStatusForAllSlaves();
        ec_close();
        return false;
    }
    yInfo("%s: ring is OPERATIONAL", Impl::kClassName.data());

    //  7. Map joints → PDO areas
    if (!m_impl->buildJointIoPointers())
    {
        ec_close();
        return false;
    }

    //  8. Fetch static SDO parameters (encoder resolutions, gear ratios)
    if (!m_impl->readEncoderResolutions())
    {
        ec_close();
        return false;
    }
    if (!m_impl->readGearRatios())
    {
        ec_close();
        return false;
    }

    // print the SDO parameters

    //  9. Start the PeriodicThread (this object) → real-time loop
    if (!this->start())
    {
        yError("%s: failed to start the thread", Impl::kClassName.data());
        ec_close();
        return false;
    }

    // 10. Resize the containers
    m_impl->variables.resizeContainers(m_impl->numAxes);

    m_impl->fillJointNames(); // fill the joint names

    yInfo("%s: opened %zu axes. Initialization complete.",
          Impl::kClassName.data(),
          m_impl->numAxes);

    return true;
}

//  close()  —  stop the thread & release the NIC
bool Ds402MotionControl::close()
{
    this->stop(); // PeriodicThread → graceful stop
    ec_close();
    yInfo("%s: EtheCAT master closed", Impl::kClassName.data());
    return true;
}

//  run()  —  gets called every period (real-time control loop)
void Ds402MotionControl::run()
{
    // 0. Request OP on first iteration --------------------------------------
    if (!m_impl->opRequested)
    {
        // 0.1. Check if the ring is still OPERATIONAL
        std::memset(m_impl->ioMap, 0, sizeof(m_impl->ioMap));
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        for (std::size_t i = 1; i <= ec_slavecount; ++i)
        {
            ec_slave[i].state = EC_STATE_OPERATIONAL;
        }
        ec_writestate(0);
        m_impl->opRequested = true;
        yInfo("%s: requested OPERATIONAL state. This is the first run() call.",
              Impl::kClassName.data());
        return; // skip one cycle to let drives switch
    }

    // 1. Cyclic exchange -----------------------------------------------------
    ec_send_processdata();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);

    if (wkc < static_cast<int>(m_impl->expectedWkc))
    {
        yWarning("%s: WKC %d < expected %d", Impl::kClassName.data(), wkc, m_impl->expectedWkc);
    }

    // 2. Read feedback from the drives --------------------------------------
    m_impl->readFeedback();
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
    // during the lifetime of the objectt. For this reason we do not need to lock the mutex
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

} // namespace yarp::dev

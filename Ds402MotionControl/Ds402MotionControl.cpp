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
        return true;
    }
}; // struct Impl

//  Ds402MotionControl  —  ctor / dtor

Ds402MotionControl::Ds402MotionControl(double period, yarp::os::ShouldUseSystemClock useSysClock)
    : yarp::os::PeriodicThread(period, useSysClock)
    , m_impl(std::make_unique<Impl>())
{
}

Ds402MotionControl::Ds402MotionControl()
    : yarp::os::PeriodicThread(0.001 /*1 kHz*/, yarp::os::ShouldUseSystemClock::Yes)
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

    //  8. Fetch static SDO parameters (encoder resolutions …)
    if (!m_impl->readEncoderResolutions())
    {
        ec_close();
        return false;
    }

    //  9. Start the PeriodicThread (this object) → real-time loop
    if (!this->start())
    {
        yError("%s: failed to start the thread", Impl::kClassName.data());
        ec_close();
        return false;
    }

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

    // 2. Read & print encoder feedback --------------------------------------
    for (size_t j = 0; j < m_impl->numAxes; ++j)
    {
        const TxPDO* fb = m_impl->tx[j];
        if (!fb)
        {
            continue;
        }

        double posDeg = 0.0;
        double velDeg = 0.0;
        if (m_impl->encoderRes[j] != 0)
        {
            posDeg = static_cast<double>(fb->PositionValue) * 360.0 / m_impl->encoderRes[j];
        }
        velDeg = static_cast<double>(fb->VelocityValue) * Impl::RPM_TO_DEG_PER_SEC;

        yInfo() << "J" << j << ": Pos " << posDeg << "°  Vel " << velDeg << "°/s  Tq "
                << fb->TorqueValue << " Nm (to check)";
    }
}

} // namespace yarp::dev

// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "EthercatManager.h"

// std
#include <chrono>
#include <thread>
// yarp
#include <yarp/os/LogStream.h>

using namespace CiA402;

constexpr uint32_t mapEntry(uint16_t idx, uint8_t sub, uint8_t bits)
{
    return (static_cast<uint32_t>(idx) << 16) | (static_cast<uint32_t>(sub) << 8) | bits;
}

EthercatManager::EthercatManager() = default;

EthercatManager::~EthercatManager()
{
    m_runWatch = false;
    if (m_watchThread.joinable())
    {
        m_watchThread.join();
    }
    if (m_initialized)
    {
        ec_close();
    }
}

static inline int wr8(int s, uint16_t idx, uint8_t sub, uint8_t v)
{
    return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
}
static inline int wr16(int s, uint16_t idx, uint8_t sub, uint16_t v)
{
    return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
}
static inline int wr32(int s, uint16_t idx, uint8_t sub, uint32_t v)
{
    return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
}

EthercatManager::Error EthercatManager::configurePDOMapping(int s)
{
    // probe for optional fields
    auto probe32 = [&](uint16_t idx, uint8_t sub) -> bool {
        int32_t tmp = 0;
        int size = sizeof(tmp);
        int rc = ec_SDOread(s, idx, sub, FALSE, &size, &tmp, EC_TIMEOUTRXM);
        return rc > 0;
    };
    const bool hasEnc1Pos = probe32(0x2111, 0x02);
    const bool hasEnc1Vel = probe32(0x2111, 0x03);
    const bool hasEnc2Pos = probe32(0x2113, 0x02);
    const bool hasEnc2Vel = probe32(0x2113, 0x03);
    const bool hasTime = probe32(0x20F0, 0x00);
    const bool hasSafe = probe32(0x6621, 0x01) && probe32(0x6621, 0x02);

    // clear existing assignments
    wr8(s, 0x1C12, 0x00, 0);
    wr8(s, 0x1C13, 0x00, 0);
    wr8(s, 0x1600, 0x00, 0);
    wr8(s, 0x1A00, 0x00, 0);

    // RxPDO 0x1600 (fixed)
    uint8_t n = 0;
    wr32(s, 0x1600, ++n, mapEntry(0x6040, 0x00, 16));
    wr32(s, 0x1600, ++n, mapEntry(0x6060, 0x00, 8));
    wr32(s, 0x1600, ++n, mapEntry(0x6071, 0x00, 16));
    wr32(s, 0x1600, ++n, mapEntry(0x607A, 0x00, 32));
    wr32(s, 0x1600, ++n, mapEntry(0x60FF, 0x00, 32));
    wr8(s, 0x1600, 0x00, n);

    // TxPDO 0x1A00 (dynamic)
    uint8_t m = 0;
    uint32_t byteOff = 0;
    auto add = [&](TxField id, uint16_t idx, uint8_t sub, uint8_t bits) {
        wr32(s, 0x1A00, ++m, mapEntry(idx, sub, bits));
        m_txMap[s - 1][id] = FieldInfo{id, byteOff, bits};
        byteOff += bits / 8;
    };

    m_txMap[s - 1].clear();
    add(TxField::Statusword, 0x6041, 0x00, 16);
    add(TxField::OpModeDisplay, 0x6061, 0x00, 8);
    add(TxField::Position6064, 0x6064, 0x00, 32);
    add(TxField::Velocity606C, 0x606C, 0x00, 32);
    add(TxField::Torque6077, 0x6077, 0x00, 16);
    add(TxField::PositionError6065, 0x6065, 0x00, 32);

    if (hasTime)
    {
        add(TxField::Timestamp20F0, 0x20F0, 0x00, 32);
    }
    if (hasSafe)
    {
        add(TxField::STO_6621_01, 0x6621, 0x01, 8);
        add(TxField::SBC_6621_02, 0x6621, 0x02, 8);
    }
    if (hasEnc1Pos)
        add(TxField::Enc1Pos2111_02, 0x2111, 0x02, 32);
    if (hasEnc1Vel)
        add(TxField::Enc1Vel2111_03, 0x2111, 0x03, 32);
    if (hasEnc2Pos)
        add(TxField::Enc2Pos2113_02, 0x2113, 0x02, 32);
    if (hasEnc2Vel)
        add(TxField::Enc2Vel2113_03, 0x2113, 0x03, 32);

    wr8(s, 0x1A00, 0x00, m);

    // Assign to SM
    wr16(s, 0x1C12, 1, 0x1600);
    wr16(s, 0x1C13, 1, 0x1A00);
    wr8(s, 0x1C12, 0x00, 1);
    wr8(s, 0x1C13, 0x00, 1);

    return Error::NoError;
}

EthercatManager::Error EthercatManager::init(const std::string& ifname) noexcept
{
    std::lock_guard<std::mutex> lk(m_ioMtx);

    if (m_initialized)
    {
        return Error::AlreadyInitialized;
    }

    yInfo("EtherCAT: init on %s", ifname.c_str());
    if (ec_init(ifname.c_str()) <= 0)
    {
        return Error::InitFailed;
    }

    /* ---------- discover slaves (PRE-OP) ---------- */
    if (ec_config_init(false) <= 0)
    {
        ec_close();
        return Error::NoSlavesFound;
    }
    yInfo("found %d slaves", ec_slavecount);

    /* ---------- (re)build PDO mapping while slaves are in PRE-OP ---------- */
    for (int s = 1; s <= ec_slavecount; ++s)
    {
        if (configurePDOMapping(s) != Error::NoError)
        {
            ec_close();
            return Error::ConfigFailed;
        }
    }
    if (ec_config_map(m_ioMap) <= 0)
    {
        ec_close();
        return Error::ConfigFailed;
    }
    ec_configdc(); // optional, keep if you use DC-sync

    /* --------------------------------------------------------------------- */
    /*  SAFE-OP transition                                                   */
    /* --------------------------------------------------------------------- */
    constexpr std::size_t ALL = 0; // 0 == broadcast node
    ec_slave[ALL].state = EC_STATE_SAFE_OP; // ▲ request SAFE-OP for everyone
    ec_writestate(ALL);
    ec_statecheck(ALL, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    if (ec_slave[ALL].state != EC_STATE_SAFE_OP)
    {
        yError("Ring failed to reach SAFE-OP (AL-status 0x%04x)", ec_slave[ALL].ALstatuscode);
        ec_close();
        return Error::SlavesNotOp;
    }

    /*  One dummy PDO round – many drives validate the mapping here  ▲ */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* --------------------------------------------------------------------- */
    /*  OPERATIONAL transition                                               */
    /* --------------------------------------------------------------------- */
    ec_slave[ALL].state = EC_STATE_OPERATIONAL; // ▲ request OP
    ec_writestate(ALL);

    /* poll until OP or timeout (≈ 10 s) */
    std::size_t attempts = 200;
    const int pollTimeout = 50'000; // 50 ms
    do
    {
        ec_statecheck(ALL, EC_STATE_OPERATIONAL, pollTimeout);
    } while (attempts-- && ec_slave[ALL].state != EC_STATE_OPERATIONAL);

    if (ec_slave[ALL].state != EC_STATE_OPERATIONAL)
    {
        yError("Ring failed to reach OP (AL-status 0x%04x)", ec_slave[ALL].ALstatuscode);
        ec_close();
        return Error::SlavesNotOp;
    }

    /* ---------- final book-keeping -------------------------------------- */
    m_expectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC; // ▲ after OP

    m_rxPtr.assign(ec_slavecount, nullptr);
    m_txRaw.assign(ec_slavecount, nullptr);
    m_txMap.assign(ec_slavecount, {});

    for (int s = 1; s <= ec_slavecount; ++s)
    {
        m_rxPtr[s - 1] = reinterpret_cast<RxPDO*>(ec_slave[s].outputs);
        m_txRaw[s - 1] = reinterpret_cast<uint8_t*>(ec_slave[s].inputs);
    }

    m_runWatch = true;
    m_watchThread = std::thread(&EthercatManager::errorMonitorLoop, this);
    m_initialized = true;

    yInfo("EtherCAT: ring is OPERATIONAL");
    return Error::NoError;
}

EthercatManager::Error EthercatManager::sendReceive() noexcept
{
    if (!m_initialized)
        return Error::NotInitialized;

    std::lock_guard<std::mutex> lk(m_ioMtx);
    ec_send_processdata();
    m_lastWkc = ec_receive_processdata(EC_TIMEOUTRET);
    return (m_lastWkc >= m_expectedWkc) ? Error::NoError : Error::PdoExchangeFailed;
}

const RxPDO* EthercatManager::getRxPDO(int slaveIdx) const noexcept
{
    return indexValid(slaveIdx) ? m_rxPtr[slaveIdx - 1] : nullptr;
}
RxPDO* EthercatManager::getRxPDO(int slaveIdx) noexcept
{
    return indexValid(slaveIdx) ? m_rxPtr[slaveIdx - 1] : nullptr;
}

TxView EthercatManager::getTxView(int slaveIdx) const noexcept
{
    if (!indexValid(slaveIdx))
    {
        return TxView(nullptr, nullptr);
    }
    return TxView(m_txRaw[slaveIdx - 1], &m_txMap[slaveIdx - 1]);
}

void EthercatManager::errorMonitorLoop() noexcept
{
    using namespace std::chrono_literals;
    while (m_runWatch.load())
    {
        {
            std::lock_guard<std::mutex> lk(m_ioMtx);
            ec_readstate();
            for (int s = 1; s <= ec_slavecount; ++s)
            {
                if (ec_slave[s].state != EC_STATE_OPERATIONAL)
                {
                    ec_slave[s].state = EC_STATE_OPERATIONAL;
                    ec_writestate(s);
                }
            }
        }
        std::this_thread::sleep_for(10ms);
    }
}
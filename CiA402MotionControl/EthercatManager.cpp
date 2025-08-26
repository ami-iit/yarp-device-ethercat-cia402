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
    auto wr8 = [s](uint16_t idx, uint8_t sub, uint8_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr16 = [s](uint16_t idx, uint8_t sub, uint16_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr32 = [s](uint16_t idx, uint8_t sub, uint32_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };

    // --------- clear existing assignments ----------
    wr8(0x1C12, 0x00, 0);
    wr8(0x1C13, 0x00, 0);
    for (uint16_t p = 0x1600; p <= 0x160F; ++p)
        wr8(p, 0x00, 0);
    for (uint16_t p = 0x1A00; p <= 0x1A0F; ++p)
        wr8(p, 0x00, 0);

    // --------- RxPDO 0x1600 (fixed) ----------
    uint8_t rx_n = 0;
    wr32(0x1600, ++rx_n, mapEntry(0x6040, 0x00, 16));
    wr32(0x1600, ++rx_n, mapEntry(0x6060, 0x00, 8));
    wr32(0x1600, ++rx_n, mapEntry(0x6071, 0x00, 16));
    wr32(0x1600, ++rx_n, mapEntry(0x607A, 0x00, 32));
    wr32(0x1600, ++rx_n, mapEntry(0x60FF, 0x00, 32));
    wr8(0x1600, 0x00, rx_n);

    // --------- TxPDO builder with rollover ----------
    constexpr int MAX_ENTRIES_PER_PDO = 8; // conservative, many drives limit to 8
    uint32_t byteOff = 0; // cumulative offset into input image
    m_txMap[s - 1].clear();

    uint16_t curTxIdx = 0x1A00;
    uint8_t curCount = 0;
    std::vector<uint16_t> txUsed;
    txUsed.reserve(4);

    auto beginTx = [&]() {
        curCount = 0;
        if (wr8(curTxIdx, 0x00, 0) <= 0)
            return false;
        txUsed.push_back(curTxIdx);
        return true;
    };
    auto finalizeTx = [&]() { return wr8(curTxIdx, 0x00, curCount) > 0; };
    auto nextTx = [&]() {
        if (!finalizeTx())
            return false;
        ++curTxIdx; // 0x1A01, 0x1A02, ...
        return beginTx();
    };
    auto ensureCap = [&]() {
        if (curCount >= MAX_ENTRIES_PER_PDO)
            return nextTx();
        return true;
    };
    auto try_map = [&](TxField id, uint16_t idx, uint8_t sub, uint8_t bits) {
        if (!ensureCap())
            return false;
        if (wr32(curTxIdx, ++curCount, mapEntry(idx, sub, bits)) <= 0)
        {
            // mapping rejected by slave → rollback this subindex number
            --curCount;
            return false;
        }
        m_txMap[s - 1][id] = FieldInfo{id, byteOff, bits};
        byteOff += bits / 8;
        return true;
    };

    // Start first TxPDO
    if (!beginTx())
        return Error::ConfigFailed;

    // --------- Mandatory CiA-402 fields (keep in front) ----------
    try_map(TxField::Statusword, 0x6041, 0x00, 16);
    try_map(TxField::OpModeDisplay, 0x6061, 0x00, 8);
    try_map(TxField::Position6064, 0x6064, 0x00, 32);
    try_map(TxField::Velocity606C, 0x606C, 0x00, 32);
    try_map(TxField::Torque6077, 0x6077, 0x00, 16);
    try_map(TxField::PositionError6065, 0x6065, 0x00, 32);

    // --------- Optional/vendor fields (spill to 0x1A01, 0x1A02 … as needed) ----------
    // (No pre-probing — we "map-by-trying". If a mapping fails, we just skip it.)
    try_map(TxField::Timestamp20F0, 0x20F0, 0x00, 32);
    try_map(TxField::STO_6621_01, 0x6621, 0x01, 8);
    try_map(TxField::SBC_6621_02, 0x6621, 0x02, 8);
    try_map(TxField::Enc1Pos2111_02, 0x2111, 0x02, 32);
    try_map(TxField::Enc1Vel2111_03, 0x2111, 0x03, 32);
    try_map(TxField::Enc2Pos2113_02, 0x2113, 0x02, 32);
    try_map(TxField::Enc2Vel2113_03, 0x2113, 0x03, 32);

    if (!finalizeTx())
        return Error::ConfigFailed;

    // --------- Assign PDOs to SyncManagers ----------
    // Rx: SM2 (0x1C12)
    wr16(0x1C12, 1, 0x1600);
    wr8(0x1C12, 0x00, 1);

    // Tx: SM3 (0x1C13) — write all we used
    wr8(0x1C13, 0x00, 0);
    for (size_t k = 0; k < txUsed.size(); ++k)
        wr16(0x1C13, static_cast<uint8_t>(k + 1), txUsed[k]);
    wr8(0x1C13, 0x00, static_cast<uint8_t>(txUsed.size()));

    yInfo("Slave %d '%s': configured %d RxPDOs, %d TxPDOs (%d bytes input)",
          s,
          ec_slave[s].name,
          1,
          static_cast<int>(txUsed.size()),
          static_cast<int>(byteOff));

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

    // Make sure slaves are in PRE-OP and support CoE/SDO before SDO traffic
    for (int s = 1; s <= ec_slavecount; ++s)
    {
        ec_statecheck(s, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
        if (!(ec_slave[s].mbx_proto & ECT_MBXPROT_COE))
        {
            yError("Slave %d '%s' has no CoE mailbox → cannot use SDO", s, ec_slave[s].name);
            ec_close();
            return Error::ConfigFailed;
        }
        if (!(ec_slave[s].CoEdetails & ECT_COEDET_SDO))
        {
            yError("Slave %d '%s' has no SDO support", s, ec_slave[s].name);
            ec_close();
            return Error::ConfigFailed;
        }
    }

    m_rxPtr.assign(ec_slavecount, nullptr);
    m_txRaw.assign(ec_slavecount, nullptr);
    m_txMap.assign(ec_slavecount, {});

    /* ---------- (re)build PDO mapping while slaves are in PRE-OP ---------- */
    for (int s = 1; s <= ec_slavecount; ++s)
    {
        yInfo("configuring slave %d: %s", s, ec_slave[s].name);

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
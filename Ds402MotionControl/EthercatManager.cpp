// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "EthercatManager.h"

// std
#include <chrono>
#include <thread>
// yarp
#include <yarp/os/LogStream.h>

using namespace Cia402;

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

EthercatManager::Error EthercatManager::configurePDOMapping(int s)
{
    auto wr8 = [s](uint16_t idx, uint8_t sub, uint8_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr32 = [s](uint16_t idx, uint8_t sub, uint32_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr16 = [s](uint16_t idx, uint8_t sub, uint16_t v) {
        return ec_SDOwrite(s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };

    /* ---------- disable existing assignments ---------- */
    wr8(0x1C12, 0x00, 0);
    wr8(0x1C13, 0x00, 0);
    wr8(0x1600, 0x00, 0);
    wr8(0x1A00, 0x00, 0);

    /* ---------- build RxPDO 0x1600 ---------- */
    uint8_t n = 0;
    wr32(0x1600, ++n, mapEntry(0x6040, 0x00, 16));
    wr32(0x1600, ++n, mapEntry(0x6060, 0x00, 8));
    wr32(0x1600, ++n, mapEntry(0x6071, 0x00, 16));
    wr32(0x1600, ++n, mapEntry(0x607A, 0x00, 32));
    wr32(0x1600, ++n, mapEntry(0x60FF, 0x00, 32));
    wr8(0x1600, 0x00, n); // sub-0 = number of mapped objects

    /* ---------- build TxPDO 0x1A00 ---------- */
    n = 0;
    wr32(0x1A00, ++n, mapEntry(0x6041, 0x00, 16));
    wr32(0x1A00, ++n, mapEntry(0x6061, 0x00, 8));
    wr32(0x1A00, ++n, mapEntry(0x6064, 0x00, 32));
    wr32(0x1A00, ++n, mapEntry(0x606C, 0x00, 32));
    wr32(0x1A00, ++n, mapEntry(0x6077, 0x00, 16));
    wr32(0x1A00, ++n, mapEntry(0x6065, 0x00, 32));

    // The following entries are specific to the Synapticon drives
    // timestamp --> synapticon
    // https://doc.synapticon.com/circulo/sw5.1/objects_html/2xxx/20f0.html?Highlight=0x20F0
    wr32(0x1A00, ++n, mapEntry(0x20F0, 0x00, 32));
    // STO --> synapticon
    // https://doc.synapticon.com/circulo/sw5.1/objects_html/6xxx/6621.html?Highlight=0x6621
    wr32(0x1A00, ++n, mapEntry(0x6621, 0x01, 8));
    // SBC  --> synapticon
    // https://doc.synapticon.com/circulo/sw5.1/objects_html/6xxx/6621.html?Highlight=0x6621
    wr32(0x1A00, ++n, mapEntry(0x6621, 0x02, 8));
    wr8(0x1A00, 0x00, n);

    /* ---------- (re)assign to sync-managers ---------- */
    uint16_t pdo = 0x1600;
    wr16(0x1C12, 1, pdo);
    pdo = 0x1A00;
    wr16(0x1C13, 1, pdo);
    wr8(0x1C12, 0x00, 1); // sub-0 = #PDOs
    wr8(0x1C13, 0x00, 1);

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
    m_txPtr.assign(ec_slavecount, nullptr);
    for (int s = 1; s <= ec_slavecount; ++s)
    {
        m_rxPtr[s - 1] = reinterpret_cast<RxPDO*>(ec_slave[s].outputs);
        m_txPtr[s - 1] = reinterpret_cast<TxPDO*>(ec_slave[s].inputs);
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
const TxPDO* EthercatManager::getTxPDO(int slaveIdx) const noexcept
{
    return indexValid(slaveIdx) ? m_txPtr[slaveIdx - 1] : nullptr;
}
TxPDO* EthercatManager::getTxPDO(int slaveIdx) noexcept
{
    return indexValid(slaveIdx) ? m_txPtr[slaveIdx - 1] : nullptr;
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
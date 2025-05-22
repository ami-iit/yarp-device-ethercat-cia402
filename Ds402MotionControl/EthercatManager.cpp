#include "EthercatManager.h"
#include <chrono>
#include <thread>

using namespace Cia402;

EthercatManager::EthercatManager() = default;

EthercatManager::~EthercatManager()
{
    m_runWatch = false;
    if (m_watchThread.joinable())
        m_watchThread.join();
    if (m_initialized)
        ec_close();
}

EthercatManager::Error EthercatManager::init(const std::string& ifname) noexcept
{
    std::lock_guard<std::mutex> lk(m_ioMtx);

    if (m_initialized)
        return Error::AlreadyInitialized;

    if (ec_init(ifname.c_str()) <= 0)
        return Error::InitFailed;

    if (ec_config_init(false) <= 0)
    {
        ec_close();
        return Error::NoSlavesFound;
    }

    if (ec_config_map(m_ioMap) <= 0)
    {
        ec_close();
        return Error::ConfigFailed;
    }
    ec_configdc();

    m_expectedWkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;

    m_rxPtr.assign(ec_slavecount, nullptr);
    m_txPtr.assign(ec_slavecount, nullptr);
    for (int s = 1; s <= ec_slavecount; ++s)
    {
        m_rxPtr[s - 1] = reinterpret_cast<RxPDO*>(ec_slave[s].outputs);
        m_txPtr[s - 1] = reinterpret_cast<TxPDO*>(ec_slave[s].inputs);
    }

    for (int s = 1; s <= ec_slavecount; ++s)
        ec_slave[s].state = EC_STATE_OPERATIONAL;

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    int timeoutCtr = 200;
    while (timeoutCtr-- && ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    }

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        ec_close();
        return Error::SlavesNotOp;
    }

    m_runWatch = true;
    m_watchThread = std::thread(&EthercatManager::errorMonitorLoop, this);
    m_initialized = true;
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
RxPDO* EthercatManager::getRxPDOMutable(int slaveIdx) noexcept
{
    return indexValid(slaveIdx) ? m_rxPtr[slaveIdx - 1] : nullptr;
}
const TxPDO* EthercatManager::getTxPDO(int slaveIdx) const noexcept
{
    return indexValid(slaveIdx) ? m_txPtr[slaveIdx - 1] : nullptr;
}
TxPDO* EthercatManager::getTxPDOMutable(int slaveIdx) noexcept
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
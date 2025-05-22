// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_CIA402_ETHERCAT_MANAGER_H
#define YARP_DEV_CIA402_ETHERCAT_MANAGER_H

#include "ethercat.h"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace Cia402
{

#pragma pack(push, 1)
struct TxPDO // (= slave → master, also called *Tx*PDO)
{
    uint16_t Statusword;
    int8_t OpModeDisplay;
    int32_t PositionValue; // [encoder counts]
    int32_t VelocityValue; // [rpm]
    int16_t TorqueValue; // [0.1 Nm]
    int32_t PositionErrorActualValue;
    uint32_t Timestamp; // [us] --> Synapticon only
    uint8_t STO; // --> Synapticon only
    uint8_t SBC; // --> Synapticon only
};

struct RxPDO // (= master → slave, also called *Rx*PDO)
{
    uint16_t Controlword;
    int8_t OpMode;
    int16_t TargetTorque;
    int32_t TargetPosition;
    int32_t TargetVelocity;
};
#pragma pack(pop)

class EthercatManager
{
public:
    enum class Error : int
    {
        NoError = 0,
        InitFailed = -1,
        NoSlavesFound = -2,
        ConfigFailed = -3,
        SlavesNotOp = -4,
        AlreadyInitialized = -5,
        NotInitialized = -6,
        InvalidSlaveIndex = -7,
        PdoExchangeFailed = -8,
    };

    EthercatManager();
    ~EthercatManager();

    EthercatManager(const EthercatManager&) = delete;
    EthercatManager& operator=(const EthercatManager&) = delete;

    Error init(const std::string& ifname) noexcept;
    bool isInitialized() const noexcept
    {
        return m_initialized;
    }

    Error sendReceive() noexcept;
    int getWorkingCounter() const noexcept
    {
        return m_lastWkc;
    }

    const RxPDO* getRxPDO(int slaveIndex) const noexcept;
    RxPDO* getRxPDO(int slaveIndex) noexcept;
    const TxPDO* getTxPDO(int slaveIndex) const noexcept;
    TxPDO* getTxPDO(int slaveIndex) noexcept;

    template <typename T>
    Error readSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, T& out) noexcept;

private:
    void errorMonitorLoop() noexcept;
    bool indexValid(int idx) const noexcept
    {
        return idx >= 1 && idx <= ec_slavecount;
    }

    Error configurePDOMapping(int s);

    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_runWatch{false};
    std::thread m_watchThread;

    std::vector<RxPDO*> m_rxPtr;
    std::vector<TxPDO*> m_txPtr;

    int m_lastWkc{0};
    int m_expectedWkc{0};
    char m_ioMap[4096]{};

    mutable std::mutex m_ioMtx;
};

template <typename T>
EthercatManager::Error
EthercatManager::readSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, T& out) noexcept
{
    if (!m_initialized.load())
        return Error::NotInitialized;
    if (!indexValid(slaveIndex))
        return Error::InvalidSlaveIndex;

    int size = sizeof(T);
    int rc = 0;
    {
        std::lock_guard<std::mutex> lk(m_ioMtx);
        rc = ec_SDOread(slaveIndex, idx, subIdx, false, &size, &out, EC_TIMEOUTRXM);
    }
    return (rc > 0) ? Error::NoError : Error::PdoExchangeFailed;
}

} // namespace Cia402

#endif // YARP_DEV_CIA402_ETHERCAT_MANAGER_H

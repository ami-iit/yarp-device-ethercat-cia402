// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "EthercatManager.h"

// std
#include <chrono>
#include <thread>
// yarp
#include <yarp/os/LogStream.h>

using namespace CiA402;

/**
 * @brief Helper function to create a PDO mapping entry.
 *
 * EtherCAT PDO mappings use a 32-bit value that encodes:
 * - Bits 31-16: Object index (e.g., 0x6041 for statusword)
 * - Bits 15-8:  Object subindex (usually 0x00)
 * - Bits 7-0:   Size in bits (8, 16, 32, etc.)
 *
 * @param idx Object index (16 bits)
 * @param sub Object subindex (8 bits)
 * @param bits Size of the object in bits (8 bits)
 * @return 32-bit mapping entry for use in PDO configuration
 */
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

    // disable the synchronized clock
    this->disableDCSync0();

    // Close only if port was opened
    if (m_portOpen)
    {
        ecx_close(&m_ctx);
    }
}

EthercatManager::Error EthercatManager::configurePDOMapping(int s)
{
    // Helper lambdas to write SDO values of different sizes to slave 's'
    // These wrap ec_SDOwrite with proper type casting and timeout handling
    auto wr8 = [this, s](uint16_t idx, uint8_t sub, uint8_t v) {
        return ecx_SDOwrite(&m_ctx, s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr16 = [this, s](uint16_t idx, uint8_t sub, uint16_t v) {
        return ecx_SDOwrite(&m_ctx, s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };
    auto wr32 = [this, s](uint16_t idx, uint8_t sub, uint32_t v) {
        return ecx_SDOwrite(&m_ctx, s, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    };

    // --------- Clear existing PDO assignments ----------
    // Before configuring new mappings, we must clear all existing ones
    // 0x1C12: RxPDO assignment (master → slave data)
    // 0x1C13: TxPDO assignment (slave → master data)
    wr8(0x1C12, 0x00, 0); // Clear RxPDO SyncManager assignment
    wr8(0x1C13, 0x00, 0); // Clear TxPDO SyncManager assignment

    // Clear all RxPDO mapping objects (0x1600-0x160F)
    for (uint16_t p = 0x1600; p <= 0x160F; ++p)
        wr8(p, 0x00, 0);

    // Clear all TxPDO mapping objects (0x1A00-0x1A0F)
    for (uint16_t p = 0x1A00; p <= 0x1A0F; ++p)
        wr8(p, 0x00, 0);

    // --------- Configure RxPDO 0x1600 (master → slave, fixed mapping) ----------
    // RxPDO contains the control data we send to the drive every cycle
    // This mapping is fixed because all CiA402 drives support these standard objects
    uint8_t rx_n = 0; // Entry counter for RxPDO 0x1600

    // Map standard CiA402 control objects into RxPDO 0x1600:
    wr32(0x1600, ++rx_n, mapEntry(0x6040, 0x00, 16)); // Controlword (16 bits)
    wr32(0x1600, ++rx_n, mapEntry(0x6060, 0x00, 8)); // Modes of operation (8 bits)
    wr32(0x1600, ++rx_n, mapEntry(0x6071, 0x00, 16)); // Target torque (16 bits)
    wr32(0x1600, ++rx_n, mapEntry(0x607A, 0x00, 32)); // Target position (32 bits)
    wr32(0x1600, ++rx_n, mapEntry(0x60FF, 0x00, 32)); // Target velocity (32 bits)

    // Write the total number of entries to finalize the RxPDO mapping
    wr8(0x1600, 0x00, rx_n);

    // --------- Dynamic TxPDO builder with automatic rollover ----------
    // TxPDO contains feedback data from slave to master (status, positions, etc.)
    // Unlike RxPDO, TxPDO mapping is dynamic because:
    // 1. Not all drives support all optional objects (vendor-specific encoders, safety signals)
    // 2. PDOs have size limits (~8 entries), so we may need multiple TxPDOs
    // 3. We want to gracefully handle mapping failures for optional fields

    constexpr int MAX_ENTRIES_PER_PDO = 8; // Conservative limit, many drives restrict to 8 entries
                                           // per PDO
    uint32_t byteOff = 0; // Cumulative byte offset in the TxPDO process image
    m_txMap[s - 1].clear(); // Clear any existing field mapping for this slave

    // Current TxPDO being built (starts with 0x1A00, then 0x1A01, 0x1A02, etc.)
    uint16_t curTxIdx = 0x1A00;
    uint8_t curCount = 0; // Number of entries in current TxPDO
    std::vector<uint16_t> txUsed; // Track which TxPDO indices we actually used
    txUsed.reserve(4); // Expect to use at most a few TxPDOs

    // Lambda to start building a new TxPDO (clear entry count, add to used list)
    auto beginTx = [&]() {
        curCount = 0;
        if (wr8(curTxIdx, 0x00, 0) <= 0) // Clear the PDO entry count
            return false;
        txUsed.push_back(curTxIdx); // Remember we're using this PDO index
        return true;
    };

    // Lambda to finalize current TxPDO (write final entry count)
    auto finalizeTx = [&]() { return wr8(curTxIdx, 0x00, curCount) > 0; };

    // Lambda to move to the next TxPDO (finalize current, increment index, begin new)
    auto nextTx = [&]() {
        if (!finalizeTx())
            return false;
        ++curTxIdx; // Move to next PDO index (0x1A01, 0x1A02, ...)
        return beginTx();
    };

    // Lambda to ensure we have capacity in current TxPDO (roll over to next if full)
    auto ensureCap = [&]() {
        if (curCount >= MAX_ENTRIES_PER_PDO)
            return nextTx();
        return true;
    };
    // Lambda to attempt mapping a field into the current TxPDO
    // This is the core mapping logic that:
    // 1. Ensures we have space in the current PDO (or creates a new one)
    // 2. Attempts to map the object/subindex into the PDO
    // 3. If successful, records the field location in our mapping table
    // 4. If failed, silently continues (graceful degradation for optional fields)
    auto tryMap = [&](TxField id, uint16_t idx, uint8_t sub, uint8_t bits) {
        if (!ensureCap()) // Make sure we have space, roll over to next PDO if needed
            return false;

        // Try to add this mapping entry to the current PDO
        if (wr32(curTxIdx, ++curCount, mapEntry(idx, sub, bits)) <= 0)
        {
            // Mapping rejected by slave (object doesn't exist or not mappable)
            // Rollback the subindex counter and continue gracefully
            --curCount;
            return false;
        }

        // Success! Record where this field is located in the process image
        m_txMap[s - 1][id] = FieldInfo{id, byteOff, bits};
        byteOff += bits / 8; // Advance byte offset for next field
        return true;
    };

    // Start building the first TxPDO (0x1A00)
    if (!beginTx())
        return Error::ConfigFailed;

    // --------- Map mandatory CiA-402 fields (highest priority) ----------
    // These fields are essential for basic CiA402 operation and should be
    // placed in the first TxPDO for optimal performance and compatibility
    tryMap(TxField::Statusword, 0x6041, 0x00, 16); // Drive status bits
    tryMap(TxField::OpModeDisplay, 0x6061, 0x00, 8); // Current operation mode
    tryMap(TxField::Position6064, 0x6064, 0x00, 32); // Actual position
    tryMap(TxField::Velocity606C, 0x606C, 0x00, 32); // Actual velocity
    tryMap(TxField::Torque6077, 0x6077, 0x00, 16); // Actual torque
    tryMap(TxField::PositionError6065, 0x6065, 0x00, 32); // Position error

    // --------- Map optional/vendor-specific fields ----------
    // These fields may not be supported by all drives, so we use tryMap
    // which gracefully handles mapping failures. If a PDO becomes full,
    // the ensureCap() logic will automatically roll over to the next PDO.

    // Optional timestamp for synchronized operation
    tryMap(TxField::Timestamp20F0, 0x20F0, 0x00, 32);

    // Safety signals mapped into PDOs for real-time access
    // NOTE: This deviates from strict CiA402 compliance but provides significant
    // advantages in fault management and control transitions
    tryMap(TxField::STO_6621_01, 0x6621, 0x01, 8); // Safe Torque Off status
    tryMap(TxField::SBC_6621_02, 0x6621, 0x02, 8); // Safe Brake Control status

    // Vendor-specific encoder feedbacks (dual encoder support)
    tryMap(TxField::Enc1Pos2111_02, 0x2111, 0x02, 32); // Encoder 1 position
    tryMap(TxField::Enc1Vel2111_03, 0x2111, 0x03, 32); // Encoder 1 velocity
    tryMap(TxField::Enc2Pos2113_02, 0x2113, 0x02, 32); // Encoder 2 position
    tryMap(TxField::Enc2Vel2113_03, 0x2113, 0x03, 32); // Encoder 2 velocity

    // temperature
    tryMap(TxField::TemperatureDrive, 0x2031, 0x01, 32); // Drive temperature

    // Finalize the last TxPDO we were building
    if (!finalizeTx())
        return Error::ConfigFailed;

    // --------- Assign PDOs to SyncManagers ----------
    // SyncManagers are hardware units in the EtherCAT slave that handle
    // the timing and synchronization of process data exchange

    // RxPDO assignment: SyncManager 2 (SM2) handles outputs (master → slave)
    // We assign our single RxPDO 0x1600 to SM2
    wr16(0x1C12, 1, 0x1600); // Assign PDO 0x1600 to SM2, entry 1
    wr8(0x1C12, 0x00, 1); // Set SM2 to have 1 PDO total

    // TxPDO assignment: SyncManager 3 (SM3) handles inputs (slave → master)
    // We assign all the TxPDOs we created (0x1A00, 0x1A01, etc.) to SM3
    wr8(0x1C13, 0x00, 0); // Clear SM3 assignment first
    for (size_t k = 0; k < txUsed.size(); ++k)
        wr16(0x1C13, static_cast<uint8_t>(k + 1), txUsed[k]); // Assign each TxPDO
    wr8(0x1C13, 0x00, static_cast<uint8_t>(txUsed.size())); // Set total count

    // Log the mapping result for debugging
    yDebug("Slave %d '%s': configured %d RxPDOs, %d TxPDOs (%d bytes input)",
           s,
           m_ctx.slavelist[s].name,
           1,
           static_cast<int>(txUsed.size()),
           static_cast<int>(byteOff));

    return Error::NoError;
}

EthercatManager::Error EthercatManager::init(const std::string& ifname) noexcept
{
    std::lock_guard<std::mutex> lk(m_ioMtx);

    // Prevent double initialization
    if (m_initialized)
    {
        return Error::AlreadyInitialized;
    }

    // --------- Initialize SOEM EtherCAT stack ----------
    // SOEM (Simple Open EtherCAT Master) is the underlying EtherCAT library
    yInfo("%s: EtherCAT: init on %s", m_kClassName.data(), ifname.c_str());
    if (ecx_init(&m_ctx, ifname.c_str()) <= 0)
    {
        return Error::InitFailed;
    }

    m_portOpen = true;

    // --------- Discover slaves and transition to PRE-OP ----------
    // This scans the EtherCAT ring and identifies all connected slaves
    // Slaves start in INIT state and are transitioned to PRE-OP for configuration
    if (ecx_config_init(&m_ctx) <= 0)
    {
        ecx_close(&m_ctx);
        m_portOpen = false;
        return Error::NoSlavesFound;
    }
    yInfo("%s: found %d slaves", m_kClassName.data(), (m_ctx.slavecount ? m_ctx.slavecount : 0));

    // --------- Validate slave capabilities ----------
    // Before we can configure PDOs via SDO, we need to ensure each slave:
    // 1. Is in PRE-OP state (required for mailbox communication)
    // 2. Supports CoE (CANopen over EtherCAT) mailbox protocol
    // 3. Supports SDO (Service Data Objects) for configuration
    for (int s = 1; s <= m_ctx.slavecount; ++s)
    {
        // Ensure slave reached PRE-OP state (required for mailbox access)
        ecx_statecheck(&m_ctx, s, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

        if (!(m_ctx.slavelist[s].mbx_proto & ECT_MBXPROT_COE))
        {
            yError("%s: Slave %d '%s' has no CoE mailbox → cannot use SDO",
                   m_kClassName.data(),
                   s,
                   m_ctx.slavelist[s].name);
            return Error::ConfigFailed;
        }

        // Check if slave supports SDO within CoE
        if (!(m_ctx.slavelist[s].CoEdetails & ECT_COEDET_SDO))
        {
            yError("%s: Slave %d '%s' has no SDO support",
                   m_kClassName.data(),
                   s,
                   m_ctx.slavelist[s].name);
            return Error::ConfigFailed;
        }
    }

    // --------- Initialize internal data structures ----------
    // Prepare containers for per-slave PDO pointers and mappings
    m_rxPtr.assign(m_ctx.slavecount, nullptr); // RxPDO pointers (master → slave)
    m_txRaw.assign(m_ctx.slavecount, nullptr); // Raw TxPDO buffers (slave → master)
    m_txMap.assign(m_ctx.slavecount, {}); // TxPDO field mappings

    // --------- Configure PDO mappings for each slave ----------
    // This is the most critical phase: we configure how process data is organized
    // in the cyclic exchange. Must be done while slaves are in PRE-OP state.
    for (int s = 1; s <= m_ctx.slavecount; ++s)
    {
        yInfo("%s: configuring slave %d: %s", m_kClassName.data(), s, m_ctx.slavelist[s].name);

        if (configurePDOMapping(s) != Error::NoError)
        {
            return Error::ConfigFailed;
        }
    }

    // --------- Build process data image ----------
    // SOEM creates a contiguous memory buffer containing all slave PDOs
    // This "IO map" is used for efficient cyclic data exchange
    if (ecx_config_map_group(&m_ctx, m_ioMap, 0) <= 0)
    {
        return Error::ConfigFailed;
    }

    // Configure distributed clocks for time synchronization (optional but recommended)
    ecx_configdc(&m_ctx);

    // =========================================================================
    // SAFE-OP STATE TRANSITION
    // =========================================================================
    // SAFE-OP is an intermediate state where:
    // - PDO configuration is locked and validated
    // - Cyclic communication is enabled but outputs are disabled for safety
    // - Allows one test cycle to validate the PDO mapping

    constexpr std::size_t ALL = 0; // 0 == broadcast to all slaves

    // Request SAFE-OP state for all slaves
    m_ctx.slavelist[ALL].state = EC_STATE_SAFE_OP;
    ecx_writestate(&m_ctx, ALL);
    ecx_statecheck(&m_ctx, ALL, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    if (m_ctx.slavelist[ALL].state != EC_STATE_SAFE_OP)
    {
        yError("%s: Ring failed to reach SAFE-OP (AL-status 0x%04x)",
               m_kClassName.data(),
               m_ctx.slavelist[ALL].ALstatuscode);
        return Error::SlavesNotOp;
    }

    // --------- Validate PDO mapping with dummy cycle ----------
    // Many drives validate their PDO mapping during the first data exchange
    // This dummy send/receive ensures the mapping is accepted before going to OP
    ecx_send_processdata(&m_ctx);
    ecx_receive_processdata(&m_ctx, EC_TIMEOUTRET);

    // =========================================================================
    // OPERATIONAL STATE TRANSITION
    // =========================================================================
    // OPERATIONAL is the final state where:
    // - All safety interlocks are released
    // - Outputs are enabled and drives can produce motion
    // - Full cyclic operation begins

    // Request OPERATIONAL state for all slaves
    m_ctx.slavelist[ALL].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&m_ctx, ALL);

    // --------- Wait for OPERATIONAL state with timeout ----------
    // Some drives take time to complete their internal initialization
    // We poll periodically until all slaves reach OP or we timeout
    std::size_t attempts = 200; // Maximum polling attempts
    const int pollTimeout = 50'000; // 50 ms per poll = ~10 second total timeout
    do
    {
        ecx_statecheck(&m_ctx, ALL, EC_STATE_OPERATIONAL, pollTimeout);
    } while (attempts-- && m_ctx.slavelist[ALL].state != EC_STATE_OPERATIONAL);

    // Check if transition was successful
    if (m_ctx.slavelist[ALL].state != EC_STATE_OPERATIONAL)
    {
        yError("%s: Ring failed to reach OP (AL-status 0x%04x)",
               m_kClassName.data(),
               m_ctx.slavelist[ALL].ALstatuscode);
        return Error::SlavesNotOp;
    }

    // =========================================================================
    // FINAL INITIALIZATION AND BOOKKEEPING
    // =========================================================================

    // --------- Calculate expected Working Counter ----------
    // Working Counter (WKC) is EtherCAT's mechanism for detecting communication errors
    // Each successful PDO exchange increments the counter by the number of slaves involved
    // We calculate the expected value based on SOEM's internal group configuration
    m_expectedWkc = m_ctx.grouplist[0].outputsWKC * 2 + m_ctx.grouplist[0].inputsWKC;

    // --------- Set up PDO access pointers ----------
    // Now that the IO map is finalized, we can cache pointers to each slave's
    // PDO areas for efficient runtime access
    for (int s = 1; s <= m_ctx.slavecount; ++s)
    {
        // Cache pointer to RxPDO (master → slave data) with proper type casting
        m_rxPtr[s - 1] = reinterpret_cast<RxPDO*>(m_ctx.slavelist[s].outputs);

        // Cache pointer to raw TxPDO buffer (slave → master data)
        // We use raw uint8_t* because TxPDO layout is dynamic and accessed via TxView
        m_txRaw[s - 1] = reinterpret_cast<uint8_t*>(m_ctx.slavelist[s].inputs);
    }

    // --------- Initialize health monitoring data ----------
    {
        std::lock_guard<std::mutex> g(m_healthMtx);
        m_notOpConsecutive.assign(m_ctx.slavecount, 0);
        m_slaveHealth.assign(m_ctx.slavecount, SlaveHealth::Ok);
    }

    // --------- Start background error monitoring ----------
    // Launch a background thread to continuously monitor slave states
    // and attempt recovery if any slave drops out of OPERATIONAL state
    m_runWatch = true;
    m_watchThread = std::thread(&EthercatManager::errorMonitorLoop, this);

    // Mark initialization as complete
    m_initialized = true;

    yInfo("%s: EtherCAT: ring is OPERATIONAL", m_kClassName.data());
    return Error::NoError;
}

EthercatManager::Error EthercatManager::sendReceive() noexcept
{
    // Ensure we are initialized before attempting communication
    if (!m_initialized)
    {
        return Error::NotInitialized;
    }
    // Perform cyclic process data exchange with thread safety
    std::lock_guard<std::mutex> lk(m_ioMtx);
    ecx_send_processdata(&m_ctx);
    m_lastWkc = ecx_receive_processdata(&m_ctx, EC_TIMEOUTRET);
    return (m_lastWkc >= m_expectedWkc) ? Error::NoError : Error::PdoExchangeFailed;
}

const RxPDO* EthercatManager::getRxPDO(int slaveIdx) const noexcept
{
    return this->indexValid(slaveIdx) ? m_rxPtr[slaveIdx - 1] : nullptr;
}

RxPDO* EthercatManager::getRxPDO(int slaveIdx) noexcept
{
    return this->indexValid(slaveIdx) ? m_rxPtr[slaveIdx - 1] : nullptr;
}

TxView EthercatManager::getTxView(int slaveIdx) const noexcept
{
    return this->indexValid(slaveIdx) ? TxView(m_txRaw[slaveIdx - 1], &m_txMap[slaveIdx - 1])
                                      : TxView(nullptr, nullptr);
}

void EthercatManager::errorMonitorLoop() noexcept
{
    using namespace std::chrono_literals;

    while (m_runWatch.load(std::memory_order_relaxed))
    {
        {
            std::lock_guard<std::mutex> ioLk(m_ioMtx);
            ecx_readstate(&m_ctx);

            std::lock_guard<std::mutex> hLk(m_healthMtx);

            for (int s = 1; s <= m_ctx.slavecount; ++s)
            {
                const bool inOp = (m_ctx.slavelist[s].state == EC_STATE_OPERATIONAL);

                if (inOp)
                {
                    m_slaveHealth[s - 1] = SlaveHealth::Ok;
                    m_notOpConsecutive[s - 1] = 0;
                    continue;
                }

                // not OP → degrade and count
                int tries = ++m_notOpConsecutive[s - 1];
                if (tries > kMaxStateRetries)
                {
                    m_slaveHealth[s - 1] = SlaveHealth::Lost;
                    continue; // stop trying
                }

                m_slaveHealth[s - 1] = SlaveHealth::Degraded;

                // one gentle attempt to request OP (unchanged policy)
                m_ctx.slavelist[s].state = EC_STATE_OPERATIONAL;
                ecx_writestate(&m_ctx, s);
            }
        }

        std::this_thread::sleep_for(10ms);
    }
}

std::string EthercatManager::getName(int slaveIndex) const noexcept
{
    if (!indexValid(slaveIndex))
    {
        return {};
    }
    return m_ctx.slavelist[slaveIndex].name;
}

bool EthercatManager::indexValid(int slaveIndex) const noexcept
{
    return slaveIndex >= 1 && slaveIndex <= m_ctx.slavecount;
}

EthercatManager::Error EthercatManager::enableDCSync0(uint32_t cycleNs, int32_t shiftNs) noexcept
{
    if (!m_initialized)
    {
        return Error::NotInitialized;
    }

    std::lock_guard<std::mutex> lk(m_ioMtx);

    // Ensure DC is configured at bus level (you already call ecx_configdc in init)
    // Now enable SYNC0 on each slave (SYNC1 disabled)
    for (int s = 1; s <= m_ctx.slavecount; ++s)
    {
        // ecx_dcsync0(ctx, slave, activate, CyclTime, ShiftTime)
        // Master-mode behavior is handled by the master; slaves just emit SYNC0
        ecx_dcsync0(&m_ctx, s, true, cycleNs, shiftNs);
    }
    return Error::NoError;
}

EthercatManager::Error EthercatManager::disableDCSync0() noexcept
{
    if (!m_initialized)
    {
        return Error::NotInitialized;
    }
    std::lock_guard<std::mutex> lk(m_ioMtx);
    for (int s = 1; s <= m_ctx.slavecount; ++s)
    {
        ecx_dcsync0(&m_ctx, s, false, 0, 0);
    }
    return Error::NoError;
}

EthercatManager::SlaveHealth EthercatManager::getSlaveHealth(int slaveIndex) const noexcept
{
    if (!indexValid(slaveIndex))
    {
        return SlaveHealth::Lost;
    }
    std::lock_guard<std::mutex> g(m_healthMtx);
    return m_slaveHealth[slaveIndex - 1];
}

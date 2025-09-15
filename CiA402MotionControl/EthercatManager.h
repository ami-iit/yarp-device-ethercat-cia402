// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_CIA402_ETHERCAT_MANAGER_H
#define YARP_DEV_CIA402_ETHERCAT_MANAGER_H

#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// SOEM
#include <soem/soem.h>

namespace CiA402
{

#pragma pack(push, 1)

/**
 * @brief Process Data Object written from master to slave (RxPDO).
 *
 * This struct mirrors the layout of the outputs sent cyclically to each drive.
 * The exact mapping is configured at startup (see configurePDOMapping), but
 * fields correspond to standard CiA-402 objects:
 * - Controlword      (0x6040)
 * - OpMode           (0x6060)
 * - TargetTorque     (0x6071)
 * - TargetPosition   (0x607A)
 * - TargetVelocity   (0x60FF)
 */
struct RxPDO // (= master → slave, also called *Rx*PDO)
{
    uint16_t Controlword; ///< 0x6040: Controlword
    int8_t OpMode; ///< 0x6060: Modes of operation
    int16_t TargetTorque; ///< 0x6071: Target torque
    int32_t TargetPosition; ///< 0x607A: Target position
    int32_t TargetVelocity; ///< 0x60FF: Target velocity
};
#pragma pack(pop)

/**
 * @brief Identifiers for fields read from a slave (TxPDO → master).
 *
 * Each value corresponds to a standard or vendor-specific object/subindex
 * that can be mapped into the TxPDO image and accessed via TxView.
 */
enum class TxField : uint8_t
{
    Statusword, ///< 0x6041: Statusword
    OpModeDisplay, ///< 0x6061: Modes of operation display
    Position6064, ///< 0x6064: Position actual value
    Velocity606C, ///< 0x606C: Velocity actual value
    Torque6077, ///< 0x6077: Torque actual value
    PositionError6065, ///< 0x6065: Position demand value / error
    Timestamp20F0, ///< 0x20F0: Timestamp (if supported)
    STO_6621_01, ///< 0x6621:01 Safe Torque Off status (exposed via PDO)
    SBC_6621_02, ///< 0x6621:02 Safe Brake Control status (exposed via PDO)
    Enc1Pos2111_02, ///< 0x2111:02 Vendor enc1 position
    Enc1Vel2111_03, ///< 0x2111:03 Vendor enc1 velocity
    Enc2Pos2113_02, ///< 0x2113:02 Vendor enc2 position
    Enc2Vel2113_03, ///< 0x2113:03 Vendor enc2 velocity
    TemperatureDrive ///< 0x2031:01 Temperature (if supported)
};

/**
 * @brief Metadata describing a field inside the raw TxPDO image.
 */
struct FieldInfo
{
    TxField id; ///< Logical field id.
    uint32_t byteOffset; ///< Byte offset from the base of the Tx image.
    uint8_t bitSize; ///< Size in bits (8/16/32).
};

/**
 * @brief Lightweight view over a slave's TxPDO image.
 *
 * Provides presence checks and typed accessors to values mapped into the
 * device TxPDO. Lifetime note: this is a non-owning view, valid as long as
 * the underlying process image buffer remains alive and unmoved.
 */
class TxView
{
public:
    /**
     * @brief Construct a view.
     * @param base Pointer to the beginning of the raw TxPDO buffer for a slave.
     * @param map  Pointer to the field map describing where each entry is.
     */
    TxView(const uint8_t* base, const std::unordered_map<TxField, FieldInfo>* map)
        : m_base(base)
        , m_map(map)
    {
    }

    /**
     * @brief Check if a given field is available in the current mapping.
     */
    bool has(TxField f) const
    {
        return m_map && (m_map->find(f) != m_map->end());
    }

    /**
     * @brief Read a field from the TxPDO image with a typed accessor.
     *
     * Only 8/16/32-bit integral types are supported. If the field is not
     * present in the mapping, the provided fallback is returned.
     */
    template <typename T> T get(TxField f, T fallback = {}) const
    {
        auto it = m_map->find(f);
        if (it == m_map->end())
        {
            return fallback;
        }
        const auto& fi = it->second;
        const uint8_t* p = m_base + fi.byteOffset;
        if constexpr (sizeof(T) == 1)
        {
            return *reinterpret_cast<const T*>(p);
        } else if constexpr (sizeof(T) == 2)
        {
            int16_t v;
            std::memcpy(&v, p, 2);
            return static_cast<T>(v);
        } else if constexpr (sizeof(T) == 4)
        {
            int32_t v;
            std::memcpy(&v, p, 4);
            return static_cast<T>(v);
        } else
        {
            static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4, "Unsupported size");
        }
        return fallback;
    }

private:
    const uint8_t* m_base{nullptr}; ///< Raw TxPDO base pointer.
    const std::unordered_map<TxField, FieldInfo>* m_map{nullptr}; ///< Mapping of available fields.
};

/**
 * @brief Minimal EtherCAT master built on SOEM for CiA-402 devices.
 *
 * Responsibilities:
 * - Discover and initialize the bus on a given network interface.
 * - Configure PDO mappings (including safety/status signals such as STO/SBC).
 * - Maintain the cyclic exchange (send/receive) of process data.
 * - Expose convenient accessors for RxPDO (outputs) and TxView (inputs).
 * - Provide helpers for SDO reads and diagnostic information (WKC).
 * - Monitor errors in a background thread.
 *
 * Notes:
 * - SOEM uses 1-based slave indices; all methods follow that convention.
 * - Pointers/views returned remain valid until re-initialization or shutdown.
 */
class EthercatManager
{
public:
    /**
     * @brief Error codes returned by EthercatManager APIs.
     */
    enum class Error : int
    {
        NoError = 0, ///< Operation completed successfully.
        InitFailed = -1, ///< Bus/stack initialization failed.
        NoSlavesFound = -2, ///< No EtherCAT slaves were discovered.
        ConfigFailed = -3, ///< PDO/SDO configuration failed.
        SlavesNotOp = -4, ///< Slaves did not reach OP state.
        AlreadyInitialized = -5, ///< init() called more than once.
        NotInitialized = -6, ///< Operation requires a prior successful init().
        InvalidSlaveIndex = -7, ///< Provided slave index is out of range (1..ec_slavecount).
        PdoExchangeFailed = -8, ///< Failed to exchange PDOs or read SDO.
    };

    /** @brief Construct the manager (no I/O started). */
    EthercatManager();
    /** @brief Destructor, stops background monitoring and releases resources. */
    ~EthercatManager();

    EthercatManager(const EthercatManager&) = delete;
    EthercatManager& operator=(const EthercatManager&) = delete;

    /**
     * @brief Initialize the EtherCAT stack and configure slaves.
     * @param ifname Network interface name (e.g., "eth0").
     * @return Error::NoError on success, specific error otherwise.
     */
    Error init(const std::string& ifname) noexcept;

    /** @brief Whether the manager has been successfully initialized. */
    bool isInitialized() const noexcept
    {
        return m_initialized;
    }

    /**
     * @brief Perform one cyclic send/receive exchange over the bus.
     * @return Error::NoError on success; PdoExchangeFailed otherwise.
     */
    Error sendReceive() noexcept;

    /**
     * @brief Last Working Counter (WKC) observed from the previous cycle.
     */
    int getWorkingCounter() const noexcept
    {
        return m_lastWkc;
    }

    int getExpectedWorkingCounter() const noexcept
    {
        return m_expectedWkc;
    }

    /**
     * @brief Access the RxPDO (master→slave) buffer for a given slave.
     * @note Pointer remains valid until re-initialization.
     */
    const RxPDO* getRxPDO(int slaveIndex) const noexcept;
    /** @overload */
    RxPDO* getRxPDO(int slaveIndex) noexcept;

    /**
     * @brief Obtain a typed, bounds-checked view over the TxPDO (slave→master).
     */
    TxView getTxView(int slaveIndex) const noexcept;

    /**
     * @brief Read an SDO value from a slave (blocking call).
     * @tparam T Trivially copyable integral type matching the object size.
     * @param slaveIndex 1-based slave index.
     * @param idx Object index (e.g., 0x6041).
     * @param subIdx Object subindex.
     * @param out Destination variable populated on success.
     * @return Error::NoError on success; an error code otherwise.
     */
    template <typename T>
    Error readSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, T& out) noexcept;

    /**
     * @brief Write an SDO value to a slave (blocking call).
     * @tparam T Trivially copyable integral type matching the object size.
     * @param slaveIndex 1-based slave index.
     * @param idx Object index (e.g., 0x6040).
     * @param subIdx Object subindex.
     * @param value Value to write.
     * @return Error::NoError on success; an error code otherwise.
     */
    template <typename T>
    Error writeSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, const T& value) noexcept;

    /**
     * @brief Get the name of a slave device.
     * @param slaveIndex 1-based slave index.
     * @return Slave name as a string.
     */
    std::string getName(int slaveIndex) const noexcept;

    /**
     * @brief Enable DC Sync0 for a slave.
     * @param cycleNs Sync cycle time in nanoseconds.
     * @param shiftNs Sync phase shift in nanoseconds.
     * @return Error::NoError on success; an error code otherwise.
     */
    Error enableDCSync0(uint32_t cycleNs, int32_t shiftNs = 0) noexcept;

    /**
     * @brief Disable DC Sync0 for a slave.
     * @return Error::NoError on success; an error code otherwise.
     */
    Error disableDCSync0() noexcept;

private:
    /** @brief Background error/AL status monitor loop. */
    void errorMonitorLoop() noexcept;

    /**
     * @brief Validate a SOEM 1-based slave index.
     * @param slaveIndex 1-based index to validate.
     * @return True if valid (1 <= slaveIndex <= slavecount); false otherwise.
     */
    bool indexValid(int slaveIndex) const noexcept;

    /**
     * @brief Program the device PDO mapping and build Tx field tables.
     * @param s 1-based slave index.
     */
    Error configurePDOMapping(int s);

    ecx_contextt m_ctx{}; ///< SOEM context structure.
    bool m_portOpen{false}; ///< True if the network port was opened.

    std::atomic<bool> m_initialized{false}; ///< True after successful init().
    std::atomic<bool> m_runWatch{false}; ///< Controls error monitor thread.
    std::thread m_watchThread; ///< Error monitor thread.

    std::vector<RxPDO*> m_rxPtr; ///< Per-slave RxPDO pointers.
    std::vector<uint8_t*> m_txRaw; ///< Per-slave raw Tx image base pointers.
    std::vector<std::unordered_map<TxField, FieldInfo>> m_txMap; ///< Per-slave Tx field maps.

    int m_lastWkc{0}; ///< Last working counter.
    int m_expectedWkc{0}; ///< Expected working counter.
    char m_ioMap[4096]{}; ///< SOEM IO map buffer (shared).

    mutable std::mutex m_ioMtx; ///< Protects IO/SDO accesses.

    static constexpr std::string_view m_kClassName = "EthercatManager"; // Class name for logging
};

template <typename T>
EthercatManager::Error
EthercatManager::readSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, T& out) noexcept
{
    // Validate preconditions before attempting EtherCAT operation
    if (!m_initialized.load())
        return Error::NotInitialized;
    if (!indexValid(slaveIndex))
        return Error::InvalidSlaveIndex;

    int size = sizeof(T); // Expected data size for type T
    int rc = 0; // SOEM return code (>0 = success, <=0 = error)

    // SDO operations must be serialized to avoid SOEM conflicts
    {
        std::lock_guard<std::mutex> lock(m_ioMtx);
        // ec_SDOread: slaveIndex, index, subindex, complete_access, size_ptr, data_ptr, timeout
        rc = ecx_SDOread(&m_ctx, slaveIndex, idx, subIdx, false, &size, &out, EC_TIMEOUTRXM);
    }

    // Convert SOEM result to our error enum
    return (rc > 0) ? Error::NoError : Error::PdoExchangeFailed;
}

template <typename T>
EthercatManager::Error
EthercatManager::writeSDO(int slaveIndex, uint16_t idx, uint8_t subIdx, const T& value) noexcept
{
    // Validate preconditions before attempting EtherCAT operation
    if (!m_initialized.load())
        return Error::NotInitialized;
    if (!indexValid(slaveIndex))
        return Error::InvalidSlaveIndex;

    int size = sizeof(T); // Expected data size for type T
    int rc = 0; // SOEM return code (>0 = success, <=0 = error)
    {
        std::lock_guard<std::mutex> lock(m_ioMtx);

        // ec_SDOwrite: slaveIndex, index, subindex, complete_access, size, data_ptr, timeout
        rc = ecx_SDOwrite(&m_ctx,
                          slaveIndex,
                          idx,
                          subIdx,
                          false,
                          size,
                          const_cast<T*>(&value),
                          EC_TIMEOUTRXM);
    }

    // Convert SOEM result to our error enum
    return (rc > 0) ? Error::NoError : Error::PdoExchangeFailed;
}

} // namespace CiA402

#endif // YARP_DEV_CIA402_ETHERCAT_MANAGER_H

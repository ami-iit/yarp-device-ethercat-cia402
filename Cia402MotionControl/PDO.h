// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file PDO.h
 * @brief Defines the Process Data Objects (PDOs) used for communication between the master and
 * slaves in the EtherCAT network.
 *
 * This file contains the definitions of the TxPDO (slave to master) and RxPDO (master to slave)
 * structures. These structures represent the data exchanged cyclically in the EtherCAT network.
 */

#ifndef YARP_DEVICE_ETHERCAT_PDO_H
#define YARP_DEVICE_ETHERCAT_PDO_H

#include "ethercat.h"
namespace Cia402
{

#pragma pack(push, 1)
/**
 * @struct TxPDO
 * @brief Represents the data sent from the slave to the master (TxPDO).
 *
 * This structure contains the feedback data provided by the slave devices, such as status,
 * position, velocity, and torque.
 */
struct TxPDO
{
    uint16_t Statusword; ///< Status of the drive, including fault and operation state information.
    int8_t OpModeDisplay; ///< Current operation mode of the drive (e.g., position, velocity, or
                          ///< torque control).
    int32_t PositionValue; ///< Current position of the drive in encoder counts.
    int32_t VelocityValue; ///< Current velocity of the drive in revolutions per minute (rpm).
    int16_t TorqueValue; ///< Current torque of the drive in 0.1 Nm units.
    int32_t PositionErrorActualValue; ///< Difference between the target and actual position values.
    uint32_t Timestamp; ///< Timestamp in microseconds (specific to Synapticon devices).
    uint8_t STO; ///< Safe Torque Off (STO) status (specific to Synapticon devices).
    uint8_t SBC; ///< Safe Brake Control (SBC) status (specific to Synapticon devices).
};

/**
 * @struct RxPDO
 * @brief Represents the data sent from the master to the slave (RxPDO).
 *
 * This structure contains the control data sent to the slave devices, such as control commands and
 * target values.
 */
struct RxPDO
{
    uint16_t Controlword; ///< Control commands for the drive, including enabling and mode
                          ///< switching.
    int8_t OpMode; ///< Desired operation mode for the drive (e.g., position, velocity, or torque
                   ///< control).
    int16_t TargetTorque; ///< Target torque for the drive in 0.1 Nm units.
    int32_t TargetPosition; ///< Target position for the drive in encoder counts.
    int32_t TargetVelocity; ///< Target velocity for the drive in revolutions per minute (rpm).
};
#pragma pack(pop)

} // namespace Cia402

#endif // YARP_DEVICE_ETHERCAT_PDO_H

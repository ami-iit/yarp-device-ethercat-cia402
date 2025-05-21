// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
//
//
//  Cia402StateMachine                                                           │
//
//  A *header‑only* helper that drives the CiA‑402 power‑state machine and the
//  “dynamic Op‑Mode switching” logic for a single EtherCAT drive.
//
//  Usage (pseudo‑code inside your cyclic task)
//
//      Cia402StateMachine sm;
//      ...
//      sm.setRequestedOpMode(ciaOpValue);          // whenever the user changes mode
//      auto cmd = sm.update(tx.Statusword, tx.OpModeDisplay);
//      rx.Controlword = cmd.controlword;
//      if (cmd.writeOpMode) rx.OpMode = cmd.opMode;
//
//  The class keeps the power stage enabled whenever the drive is already in
//  *Operation‑enabled* and the only thing that changes is the mode (object 0x6060).
//  Otherwise it gracefully walks through Shutdown → Switched‑on → Operation
//  following the CiA‑402 specification, waiting for each state to be *reached*
//  before sending the next control‑word.
//

#ifndef YARP_DEV_CIA402_STATE_MACHINE_H
#define YARP_DEV_CIA402_STATE_MACHINE_H

#include <memory>

namespace yarp
{
namespace dev
{
namespace Cia402
{

// ────────────────────────────────────────────────────────────────────────────
//  The state‑machine class
// ────────────────────────────────────────────────────────────────────────────
class StateMachine
{
public:
    /**
     * @brief Command structure.
     *
     * This structure contains the control word, operation mode, and a flag indicating
     * if the operation mode should be written.
     */
    struct Command
    {
        uint16_t controlword; ///< Control word to send to the drive.
        int8_t opMode; ///< Operation mode to send to the drive.
        bool writeOpMode; ///< Flag indicating if the operation mode should be written.
    };

    /**
     * @brief Constructor.
     *
     * Initializes the CiA-402 state machine for a single EtherCAT drive.
     */
    StateMachine();
    /**
     * @brief Destructor.
     *
     * Cleans up the resources used by the state machine.
     */
    ~StateMachine();

    /**
     * @brief Resets the internal state machine.
     *
     * Call this after power-on or after a drive fault to reinitialize the state machine.
     */
    void reset();

    /**
     * @brief Computes the next control word and operation mode to send to the drive.
     *
     * This function should be called cyclically. It determines the correct control word and
     * (optionally) the operation mode to send, based on the current statusword and opModeDisplay.
     *
     * @param statusword     Value read from TxPDO.Statusword.
     * @param opModeDisplay  Value read from TxPDO.OpModeDisplay (object 0x6061).
     * @param opReq         Requested operation mode (object 0x6060).
     * @return Command structure containing the control word, operation mode, and a flag indicating
     * if the opMode should be written.
     */
    Command update(uint16_t statusword, int8_t opModeDisplay, int8_t opReq);

    /**
     * @brief Generates a fault reset command.
     *
     * This function generates a command to reset the fault state of the drive.
     *
     * @return Command structure containing the control word and operation mode for fault reset.
     */
    Command faultReset() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace Cia402
} // namespace dev
} // namespace yarp

#endif // YARP_DEV_CIA402_STATE_MACHINE_H
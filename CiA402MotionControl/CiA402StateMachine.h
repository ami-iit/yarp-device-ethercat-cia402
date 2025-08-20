// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
//
//
//  CiA402StateMachine                                                           │
//
//  A *header‑only* helper that drives the CiA‑402 power‑state machine and the
//  “dynamic Op‑Mode switching” logic for a single EtherCAT drive.
//
//  Usage (pseudo‑code inside your cyclic task)
//
//      CiA402StateMachine sm;
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
#include <string_view>

namespace CiA402
{

enum class State : uint8_t
{
    NotReadyToSwitchOn = 0, // 0x0000
    SwitchOnDisabled = 1, // 0x0040
    ReadyToSwitchOn = 2, // 0x0021
    SwitchedOn = 3, // 0x0023
    OperationEnabled = 4, // 0x0027
    QuickStopActive = 5, // 0x0007
    FaultReaction = 6, // 0x000F
    Fault = 7, // 0x0008
    Unknown = 8
};

/** Convert a raw status-word to the canonical CiA-402 state.
 *
 *  Only the relevant bits (0,1,2,3,4,5,6) are masked, exactly as the spec does.
 *  Patterns are taken from IEC 61800-7-204, table 13.
 */
static inline constexpr State sw_to_state(uint16_t sw)
{
    const uint16_t x = sw & 0x006F; // keep bits 0,1,2,4,5,6
    const uint16_t y = sw & 0x004F; // same, but without bit 5 (QS)

    if (x == 0x0000)
        return State::NotReadyToSwitchOn; // 0000 0000 0000 0000
    if (y == 0x0040)
        return State::SwitchOnDisabled; // 0100 0000
    if (x == 0x0021)
        return State::ReadyToSwitchOn; // 0010 0001
    if (x == 0x0023)
        return State::SwitchedOn; // 0010 0011
    if (x == 0x0027)
        return State::OperationEnabled; // 0010 0111
    if (x == 0x0007)
        return State::QuickStopActive; // 0000 0111
    if (y == 0x000F)
        return State::FaultReaction; // 0000 1111  (ignore QS)
    if (y == 0x0008)
        return State::Fault; // 0000 1000
    return State::Unknown;
}

// Only for debugging
static inline constexpr std::string_view state_to_string(State s)
{
    switch (s)
    {
    case State::SwitchOnDisabled:
        return "SwitchOnDisabled";
    case State::ReadyToSwitchOn:
        return "ReadyToSwitchOn";
    case State::SwitchedOn:
        return "SwitchedOn";
    case State::OperationEnabled:
        return "OperationEnabled";
    case State::QuickStopActive:
        return "QuickStopActive";
    case State::FaultReaction:
        return "FaultReaction";
    case State::Fault:
        return "Fault";
    default:
        return "Unknown";
    }
}

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
     * @param hwInhibit     Hardware inhibit state (true if inhibited, false otherwise).
     * @return Command structure containing the control word, operation mode, and a flag indicating
     * if the opMode should be written.
     */
    Command update(uint16_t statusword, int8_t opModeDisplay, int8_t opReq, bool hwInhibit);

    int8_t getActiveOpMode() const noexcept;

    /**
     * @brief Generates a fault reset command.
     *
     * This function generates a command to reset the fault state of the drive.
     *
     * @return Command structure containing the control word and operation mode for fault reset.
     */
    Command faultReset() noexcept;

    /**
     * Check if the operation is enabled.
     * @param sw Status word.
     * @return true if the operation is enabled, false otherwise.
     */
    static inline bool isOpEnabled(uint16_t sw) { return (sw & 0x0004) != 0; }


private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace CiA402

#endif // YARP_DEV_CIA402_STATE_MACHINE_H
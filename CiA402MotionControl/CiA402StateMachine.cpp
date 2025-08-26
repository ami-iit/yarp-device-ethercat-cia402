// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "CiA402StateMachine.h"

struct CiA402::StateMachine::Impl
{
    Impl() = default;
    ~Impl() = default;

    /// Control‑word constants (CiA‑402 table 46)
    static constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
    static constexpr uint16_t CW_SHUTDOWN = 0x0006;
    static constexpr uint16_t CW_SWITCH_ON = 0x0007;
    static constexpr uint16_t CW_ENABLE_OP = 0x000F;
    static constexpr uint16_t CW_FAULT_RST = 0x0080;
    static constexpr uint16_t CW_QUICKSTOP = 0x0002;

    int8_t activeOpEcho{0}; //!< latest OpModeDisplay from the drive
};

CiA402::StateMachine::StateMachine()
    : m_impl(std::make_unique<Impl>())
{
}

CiA402::StateMachine::~StateMachine() = default;

void CiA402::StateMachine::reset()
{
    // Reset the state machine to its initial state.
    m_impl->activeOpEcho = 0;
}

int8_t CiA402::StateMachine::getActiveOpMode() const noexcept
{
    return m_impl->activeOpEcho;
}

CiA402::StateMachine::Command CiA402::StateMachine::update(uint16_t statusword,
                                                           int8_t opModeDisplay,
                                                           int8_t opReq,
                                                           bool hwInhibit)
{
    // Decode the current power state from statusword bits
    const State state = sw_to_state(statusword);

    // Track the active operation mode only when drive is fully enabled
    if (state == State::OperationEnabled)
    {
        m_impl->activeOpEcho = opModeDisplay; // Store what the drive is actually doing
    } else
    {
        m_impl->activeOpEcho = 0; // Clear when not operational
    }

    // =============================================================================
    // SAFETY OVERRIDE: Hardware inhibit forces immediate voltage disable
    // =============================================================================
    if (hwInhibit)
    {
        // Safety systems (STO/SBC) are active - immediately disable power stage
        return {Impl::CW_DISABLE_VOLTAGE, 0, true}; // Force opmode=0 (no mode)
    }

    // =============================================================================
    // FAULT HANDLING: Disable voltage but don't write opmode (preserves fault info)
    // =============================================================================
    if (state == State::Fault)
    {
        // Drive is in fault state - disable voltage but let user call faultReset()
        return {Impl::CW_DISABLE_VOLTAGE, 0, false}; // Don't overwrite opmode
    }

    // =============================================================================
    // IDLE REQUEST: User wants to stop all motion
    // =============================================================================
    if (opReq == 0)
    {
        // Disable power stage and set "no mode" explicitly
        return {Impl::CW_DISABLE_VOLTAGE, 0, true}; // Write opmode=0 once
    }

    // =============================================================================
    // DYNAMIC OPERATION MODE SWITCHING (drive already enabled)
    // =============================================================================
    if (state == State::OperationEnabled)
    {
        // Drive is fully operational - can switch modes on-the-fly
        if (opReq != m_impl->activeOpEcho)
        {
            // Mode change requested: write new opmode while maintaining power
            return {Impl::CW_ENABLE_OP, opReq, true}; // Update opmode
        } else
        {
            // Already in requested mode: maintain current state
            return {Impl::CW_ENABLE_OP, opReq, false}; // Keep current opmode
        }
    }

    // =============================================================================
    // POWER-UP SEQUENCE: Classic CiA-402 state transitions
    // =============================================================================
    // Drive needs to go through the standard power-on sequence:
    // NotReadyToSwitchOn → SwitchOnDisabled → ReadyToSwitchOn →
    // SwitchedOn → OperationEnabled

    switch (state)
    {
    case State::SwitchOnDisabled:
        // First step: enable the control electronics (but not power stage)
        return {Impl::CW_SHUTDOWN, opReq, true}; // Go to ReadyToSwitchOn

    case State::ReadyToSwitchOn:
        // Second step: prepare for switching on the power stage
        return {Impl::CW_SWITCH_ON, opReq, false}; // Go to SwitchedOn

    case State::SwitchedOn:
        // Final step: enable the power stage and operation mode
        return {Impl::CW_ENABLE_OP, opReq, false}; // Go to OperationEnabled

    case State::QuickStopActive:
        // Drive performed emergency stop - clear the condition
        return {Impl::CW_QUICKSTOP, opReq, false}; // Clear quickstop

    default:
        // NotReadyToSwitchOn, Unknown, or other: ensure safe state
        return {Impl::CW_DISABLE_VOLTAGE, 0, true}; // Force voltage off
    }
}

/** Immediate Fault reset (equivalent to requesting VOCAB_CM_FORCE_IDLE). */
CiA402::StateMachine::Command CiA402::StateMachine::faultReset() noexcept
{
    return {Impl::CW_FAULT_RST, 0, false};
}

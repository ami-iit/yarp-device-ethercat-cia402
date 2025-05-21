// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Cia402StateMachine.h"
#include <yarp/os/LogStream.h>

namespace yarp::dev::Cia402
{

/// Control‑word constants (CiA‑402 table 46)
static constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
static constexpr uint16_t CW_SHUTDOWN = 0x0006;
static constexpr uint16_t CW_SWITCH_ON = 0x0007;
static constexpr uint16_t CW_ENABLE_OP = 0x000F;
static constexpr uint16_t CW_FAULT_RST = 0x0080;
static constexpr uint16_t CW_QUICKSTOP = 0x0002;
} // namespace yarp::dev::Cia402

struct yarp::dev::Cia402::StateMachine::Impl
{
    Impl() = default;
    ~Impl() = default;

    int8_t activeOpEcho{0}; //!< latest OpModeDisplay from the drive
};

yarp::dev::Cia402::StateMachine::StateMachine()
    : m_impl(std::make_unique<Impl>())
{
}

yarp::dev::Cia402::StateMachine::~StateMachine() = default;

void yarp::dev::Cia402::StateMachine::reset()
{
    // Reset the state machine to its initial state.
    m_impl->activeOpEcho = 0;
}

yarp::dev::Cia402::StateMachine::Command
yarp::dev::Cia402::StateMachine::update(uint16_t statusword, int8_t opModeDisplay, int8_t opReq)
{
    // Map Statusword to one of the canonical CiA‑402 states.
    const State state = sw_to_state(statusword);

    if (state == State::OperationEnabled)
    {
        m_impl->activeOpEcho = opModeDisplay;
    } else
    {
        m_impl->activeOpEcho = 0; // reset the echo
    }

    // Check for Fault: always try a fault‑reset first.
    if (state == State::Fault)
    {
        return {CW_FAULT_RST, 0, false};
    }

    // yError("Cia402::update: statusword=0x%04X → %s (req=%d, active=%d)",
    //        statusword,
    //        state_to_string(state).data(),
    //        opReq,
    //        m_impl->activeOpEcho);

    // Idle
    if (opReq == 0)
    {
        return {CW_DISABLE_VOLTAGE, 0, true}; // write Op=0 once
    }

    // ///////////////////////////////// Dynamic Op‑Mode switch //////////////////////////////
    if (state == State::OperationEnabled && opReq != m_impl->activeOpEcho)
    {
        // Seamless mode switch: keep the power stage on, just write 0x6060.
        return {CW_ENABLE_OP, opReq, true};
    }

    if (state == State::OperationEnabled && opReq == m_impl->activeOpEcho)
    {
        // Seamless mode switch: keep the power stage on, just write 0x6060.
        return {CW_ENABLE_OP, opReq, true};
    }

    // ///////////////////////////////// Classical power‑cycle path //////////////////////////

    // // Default: keep whatever you were sending, do not touch OpMode.
    // return {CW_ENABLE_OP, 0, false};

    switch (state)
    {
    case State::SwitchOnDisabled:
        // go to “ReadyToSwitchOn”
        return {CW_SHUTDOWN, opReq, true};

    case State::ReadyToSwitchOn:
        // go to “SwitchedOn”
        return {CW_SWITCH_ON, opReq, false};

    case State::SwitchedOn:
        // go to “OperationEnabled”
        return {CW_ENABLE_OP, opReq, false};

    case State::QuickStopActive:
        // clear quick‐stop
        return {CW_QUICKSTOP, opReq, false};

    default:
        // NotReadyToSwitchOn or Unknown: ensure voltage off
        return {CW_DISABLE_VOLTAGE, 0, true};
    }
}

/** Immediate Fault reset (equivalent to requesting VOCAB_CM_FORCE_IDLE). */
yarp::dev::Cia402::StateMachine::Command yarp::dev::Cia402::StateMachine::faultReset() noexcept
{
    return {CW_FAULT_RST, 0, false};
}

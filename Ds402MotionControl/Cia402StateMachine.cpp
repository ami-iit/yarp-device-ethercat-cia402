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

    /**
     * @brief State machine stage.
     *
     * This enum represents the different stages of the state machine.
     * - Idle: The state machine is idle.
     * - NeedShutdown: The state machine needs to shut down.
     * - NeedSwitchOn: The state machine needs to switch on.
     * - NeedEnable: The state machine needs to enable operation.
     */
    enum class Stage
    {
        Idle, //!< The state machine is idle.
        NeedShutdown, //!< The state machine needs to shut down.
        NeedSwitchOn, //!< The state machine needs to switch on.
        NeedEnable //!< The state machine needs to enable operation.
    };

    enum class State : uint8_t
    {
        SwitchOnDisabled = 0,
        ReadyToSwitchOn = 1,
        SwitchedOn = 2,
        OperationEnabled = 3,
        QuickStopActive = 4,
        FaultReaction = 5,
        Fault = 6,
        Unknown = 7
    };

    Stage stage{Stage::Idle}; //!< Current stage of the state machine.
    int8_t activeOpEcho{0}; //!< latest OpModeDisplay from the drive

    /// Quick helper that converts a Statusword into the CiA‑402 state.
    /// Only the 7 canonical states used in normal motion are considered.
    static inline constexpr State sw_to_state(uint16_t sw)
    {
        if ((sw & 0x004F) == 0x0040)
            return State::SwitchOnDisabled; // Switch‑on disabled
        if ((sw & 0x006F) == 0x0021)
            return State::ReadyToSwitchOn;
        if ((sw & 0x006F) == 0x0023)
            return State::SwitchedOn;
        if ((sw & 0x006F) == 0x0027)
            return State::OperationEnabled;
        if ((sw & 0x006F) == 0x0007)
            return State::QuickStopActive; // Quick‑stop active
        if ((sw & 0x004F) == 0x000F)
            return State::FaultReaction; // Fault reaction active
        if ((sw & 0x004F) == 0x0008)
            return State::Fault; // Fault
        return State::Unknown; // Unknown state
    }
};

yarp::dev::Cia402::StateMachine::StateMachine()
    : m_impl(std::make_unique<Impl>())
{
}

yarp::dev::Cia402::StateMachine::~StateMachine() = default;

void yarp::dev::Cia402::StateMachine::reset()
{
    // Reset the state machine to its initial state.
    m_impl->stage = Impl::Stage::Idle;
    m_impl->activeOpEcho = 0;
}

yarp::dev::Cia402::StateMachine::Command
yarp::dev::Cia402::StateMachine::update(uint16_t statusword, int8_t opModeDisplay, int8_t opReq)
{

    // print the input of the fuction for debugging
    yError("Cia402::StateMachine: statusword=0x%04X, opModeDisplay=%d, opReq=%d",
           statusword,
           opModeDisplay,
           opReq);

    // Cache what the drive reports back so we can detect when it changes.
    m_impl->activeOpEcho = opModeDisplay;

    // Map Statusword to one of the canonical CiA‑402 states.
    const Impl::State state = Impl::sw_to_state(statusword);

    // Check for Fault: always try a fault‑reset first.
    if (state == Impl::State::Fault)
    {
        yInfo("Cia402::StateMachine: fault detected, resetting");
        m_impl->stage = Impl::Stage::Idle; // reset the state machine
        return {CW_FAULT_RST, 0, false};
    }

    // Idle
    if (opReq == 0)
    {
        // Already voltage‑disabled → nothing to do.
        if (state == Impl::State::SwitchOnDisabled)
        {
            return {CW_DISABLE_VOLTAGE, 0, false};
        }
        // Otherwise command Disable‑voltage right away.
        return {CW_DISABLE_VOLTAGE, 0, true}; // write Op=0 once
    }

    // ///////////////////////////////// Dynamic Op‑Mode switch //////////////////////////////
    if (state == Impl::State::OperationEnabled && opReq != m_impl->activeOpEcho)
    {
        // Seamless mode switch: keep the power stage on, just write 0x6060.
        return {CW_ENABLE_OP, opReq, true};
    }

    // ///////////////////////////////// Classical power‑cycle path //////////////////////////

    yInfo("Cia402::StateMachine: state=%d, opReq=%d, activeOpEcho=%d",
          int(state),
          int(opReq),
          int(m_impl->activeOpEcho));

    switch (m_impl->stage)
    {
    case Impl::Stage::Idle:
        if (state == Impl::State::SwitchOnDisabled /*Switch‑on disabled*/ && opReq != 0)
        {
            m_impl->stage = Impl::Stage::NeedShutdown;
        }
        break;

    case Impl::Stage::NeedShutdown:
        if (state == Impl::State::SwitchOnDisabled)
        {
            m_impl->stage = Impl::Stage::NeedSwitchOn;
            return {CW_SHUTDOWN, opReq, true};
        }
        break;

    case Impl::Stage::NeedSwitchOn:
        if (state == Impl::State::ReadyToSwitchOn)
        {
            m_impl->stage = Impl::Stage::NeedEnable;
            return {CW_SWITCH_ON, opReq, false};
        }
        break;

    case Impl::Stage::NeedEnable:
        if (state == Impl::State::SwitchedOn)
        {
            m_impl->stage = Impl::Stage::Idle; // done -> back to Idle
            return {CW_ENABLE_OP, opReq, false};
        }
        break;
    }

    // Default: keep whatever you were sending, do not touch OpMode.
    return {CW_ENABLE_OP, 0, false};
}

/** Immediate Fault reset (equivalent to requesting VOCAB_CM_FORCE_IDLE). */
yarp::dev::Cia402::StateMachine::Command yarp::dev::Cia402::StateMachine::faultReset() noexcept
{
    m_impl->stage = Impl::Stage::Idle;
    return {CW_FAULT_RST, 0, false};
}

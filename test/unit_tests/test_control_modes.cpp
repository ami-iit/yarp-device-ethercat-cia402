// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file test_control_modes.cpp
 * @brief Comprehensive Catch2 test suite for CiA402MotionControl device control modes
 */

#include "catch_amalgamated.hpp"
#include "mock_yarp_vocabs.h"
#include <vector>
#include <memory>

/**
 * @brief Mock implementation of IControlMode for testing purposes
 * 
 * This class provides a testable implementation of the IControlMode interface
 * without requiring actual EtherCAT hardware. It simulates the behavior of
 * CiA402MotionControl's control mode functionality.
 */
class MockControlModeDevice : public yarp::dev::IControlMode
{
private:
    int m_numAxes;
    std::vector<int> m_controlModes;

public:
    explicit MockControlModeDevice(int numAxes) 
        : m_numAxes(numAxes), m_controlModes(numAxes, VOCAB_CM_IDLE)
    {
    }

    // IControlMode interface implementation
    bool getControlMode(int j, int* mode) override
    {
        if (j < 0 || j >= m_numAxes || mode == nullptr)
            return false;
        
        *mode = m_controlModes[j];
        return true;
    }

    bool getControlModes(int* modes) override
    {
        if (modes == nullptr)
            return false;
        
        for (int i = 0; i < m_numAxes; ++i)
            modes[i] = m_controlModes[i];
        
        return true;
    }

    bool getControlModes(const int n, const int* joints, int* modes) override
    {
        if (n <= 0 || joints == nullptr || modes == nullptr)
            return false;
        
        for (int i = 0; i < n; ++i)
        {
            if (joints[i] < 0 || joints[i] >= m_numAxes)
                return false;
            
            modes[i] = m_controlModes[joints[i]];
        }
        
        return true;
    }

    bool setControlMode(const int j, const int mode) override
    {
        if (j < 0 || j >= m_numAxes)
            return false;
        
        // Validate mode (basic validation for supported CiA402 modes)
        if (!isValidMode(mode))
            return false;
        
        m_controlModes[j] = mode;
        return true;
    }

    bool setControlModes(const int n, const int* joints, int* modes) override
    {
        if (n <= 0 || joints == nullptr || modes == nullptr)
            return false;
        
        // First validate all inputs
        for (int i = 0; i < n; ++i)
        {
            if (joints[i] < 0 || joints[i] >= m_numAxes || !isValidMode(modes[i]))
                return false;
        }
        
        // If validation passes, apply all changes
        for (int i = 0; i < n; ++i)
        {
            m_controlModes[joints[i]] = modes[i];
        }
        
        return true;
    }

    bool setControlModes(int* modes) override
    {
        if (modes == nullptr)
            return false;
        
        // First validate all modes
        for (int i = 0; i < m_numAxes; ++i)
        {
            if (!isValidMode(modes[i]))
                return false;
        }
        
        // If validation passes, apply all changes
        for (int i = 0; i < m_numAxes; ++i)
        {
            m_controlModes[i] = modes[i];
        }
        
        return true;
    }

    // Helper methods for testing
    int getNumAxes() const { return m_numAxes; }
    
private:
    bool isValidMode(int mode) const
    {
        // Based on the modes_and_setpoints.md documentation, CiA402 supports:
        // - VOCAB_CM_POSITION (Profile Position Mode - PP)
        // - VOCAB_CM_VELOCITY (Cyclic Synchronous Velocity Mode - CSV)
        // - VOCAB_CM_TORQUE (Cyclic Synchronous Torque Mode - CST)
        // - VOCAB_CM_CURRENT (also uses CST mode)
        // - VOCAB_CM_IDLE / VOCAB_CM_FORCE_IDLE (power stage disabled)
        // - VOCAB_CM_POSITION_DIRECT is NOT implemented according to docs
        
        switch (mode)
        {
            case VOCAB_CM_POSITION:
            case VOCAB_CM_VELOCITY:
            case VOCAB_CM_TORQUE:
            case VOCAB_CM_CURRENT:
            case VOCAB_CM_IDLE:
            case VOCAB_CM_FORCE_IDLE:
                return true;
            case VOCAB_CM_POSITION_DIRECT:
                return false; // Not implemented according to documentation
            default:
                return false;
        }
    }
};

// Test fixture for control mode tests
class ControlModeTestFixture
{
public:
    static constexpr int NUM_AXES = 4;
    std::unique_ptr<MockControlModeDevice> device;

    ControlModeTestFixture()
        : device(std::make_unique<MockControlModeDevice>(NUM_AXES))
    {
    }
};

TEST_CASE_METHOD(ControlModeTestFixture, "Control Mode Basic Functionality", "[control_modes][basic]")
{
    SECTION("Device initialization")
    {
        REQUIRE(device != nullptr);
        REQUIRE(device->getNumAxes() == NUM_AXES);
    }
    
    SECTION("Initial control mode state")
    {
        // All axes should start in IDLE mode
        for (int i = 0; i < NUM_AXES; ++i)
        {
            int mode;
            REQUIRE(device->getControlMode(i, &mode));
            REQUIRE(mode == VOCAB_CM_IDLE);
        }
        
        // Test getting all modes at once
        std::vector<int> modes(NUM_AXES);
        REQUIRE(device->getControlModes(modes.data()));
        for (int i = 0; i < NUM_AXES; ++i)
        {
            REQUIRE(modes[i] == VOCAB_CM_IDLE);
        }
    }
}

TEST_CASE_METHOD(ControlModeTestFixture, "Single Joint Control Mode Operations", "[control_modes][single_joint]")
{
    SECTION("Set and get control mode for individual joints")
    {
        // Test each supported mode on each axis
        const std::vector<int> supportedModes = {
            VOCAB_CM_POSITION,
            VOCAB_CM_VELOCITY,
            VOCAB_CM_TORQUE,
            VOCAB_CM_CURRENT,
            VOCAB_CM_IDLE,
            VOCAB_CM_FORCE_IDLE
        };
        
        for (int axis = 0; axis < NUM_AXES; ++axis)
        {
            for (int mode : supportedModes)
            {
                // Set the mode
                REQUIRE(device->setControlMode(axis, mode));
                
                // Verify it was set correctly
                int retrievedMode;
                REQUIRE(device->getControlMode(axis, &retrievedMode));
                REQUIRE(retrievedMode == mode);
            }
        }
    }
    
    SECTION("Invalid joint indices")
    {
        int mode;
        
        // Test negative index
        REQUIRE_FALSE(device->getControlMode(-1, &mode));
        REQUIRE_FALSE(device->setControlMode(-1, VOCAB_CM_POSITION));
        
        // Test index beyond range
        REQUIRE_FALSE(device->getControlMode(NUM_AXES, &mode));
        REQUIRE_FALSE(device->setControlMode(NUM_AXES, VOCAB_CM_POSITION));
        
        // Test way beyond range
        REQUIRE_FALSE(device->getControlMode(NUM_AXES * 10, &mode));
        REQUIRE_FALSE(device->setControlMode(NUM_AXES * 10, VOCAB_CM_POSITION));
    }
    
    SECTION("Invalid mode values")
    {
        // Test unsupported mode (POSITION_DIRECT is not implemented)
        REQUIRE_FALSE(device->setControlMode(0, VOCAB_CM_POSITION_DIRECT));
        
        // Test completely invalid mode
        REQUIRE_FALSE(device->setControlMode(0, 999999));
        REQUIRE_FALSE(device->setControlMode(0, -1));
    }
    
    SECTION("Null pointer handling")
    {
        REQUIRE_FALSE(device->getControlMode(0, nullptr));
    }
}

TEST_CASE_METHOD(ControlModeTestFixture, "Multiple Joints Control Mode Operations", "[control_modes][multiple_joints]")
{
    SECTION("Set and get control modes for all joints")
    {
        // Set all joints to different modes
        std::vector<int> targetModes = {
            VOCAB_CM_POSITION,
            VOCAB_CM_VELOCITY,
            VOCAB_CM_TORQUE,
            VOCAB_CM_CURRENT
        };
        
        REQUIRE(device->setControlModes(targetModes.data()));
        
        // Verify all modes were set correctly
        std::vector<int> retrievedModes(NUM_AXES);
        REQUIRE(device->getControlModes(retrievedModes.data()));
        
        for (int i = 0; i < NUM_AXES; ++i)
        {
            REQUIRE(retrievedModes[i] == targetModes[i]);
        }
    }
    
    SECTION("Invalid mode in array affects entire operation")
    {
        // First set all axes to a known state
        std::vector<int> initialModes(NUM_AXES, VOCAB_CM_VELOCITY);
        REQUIRE(device->setControlModes(initialModes.data()));
        
        // Try to set modes with one invalid mode
        std::vector<int> invalidModes = {
            VOCAB_CM_POSITION,
            VOCAB_CM_POSITION_DIRECT, // This should cause failure
            VOCAB_CM_TORQUE,
            VOCAB_CM_CURRENT
        };
        
        REQUIRE_FALSE(device->setControlModes(invalidModes.data()));
        
        // Verify that no modes were changed (atomic operation)
        std::vector<int> unchangedModes(NUM_AXES);
        REQUIRE(device->getControlModes(unchangedModes.data()));
        for (int i = 0; i < NUM_AXES; ++i)
        {
            REQUIRE(unchangedModes[i] == VOCAB_CM_VELOCITY);
        }
    }
    
    SECTION("Null pointer handling")
    {
        std::vector<int> modes(NUM_AXES);
        
        REQUIRE_FALSE(device->getControlModes(nullptr));
        REQUIRE_FALSE(device->setControlModes(nullptr));
    }
}

TEST_CASE_METHOD(ControlModeTestFixture, "Subset of Joints Control Mode Operations", "[control_modes][subset]")
{
    SECTION("Set and get control modes for subset of joints")
    {
        // Set initial state
        std::vector<int> initialModes(NUM_AXES, VOCAB_CM_IDLE);
        REQUIRE(device->setControlModes(initialModes.data()));
        
        // Define subset of joints to modify
        std::vector<int> joints = {0, 2}; // Test joints 0 and 2
        std::vector<int> newModes = {VOCAB_CM_POSITION, VOCAB_CM_VELOCITY};
        
        REQUIRE(device->setControlModes(joints.size(), joints.data(), newModes.data()));
        
        // Verify the subset was set correctly
        std::vector<int> retrievedModes(joints.size());
        REQUIRE(device->getControlModes(joints.size(), joints.data(), retrievedModes.data()));
        
        for (size_t i = 0; i < joints.size(); ++i)
        {
            REQUIRE(retrievedModes[i] == newModes[i]);
        }
        
        // Verify other joints were not affected
        int mode1, mode3;
        REQUIRE(device->getControlMode(1, &mode1));
        REQUIRE(device->getControlMode(3, &mode3));
        REQUIRE(mode1 == VOCAB_CM_IDLE);
        REQUIRE(mode3 == VOCAB_CM_IDLE);
    }
    
    SECTION("Invalid joint in subset affects entire operation")
    {
        // Set initial state
        std::vector<int> initialModes(NUM_AXES, VOCAB_CM_VELOCITY);
        REQUIRE(device->setControlModes(initialModes.data()));
        
        // Try to set modes with one invalid joint index
        std::vector<int> invalidJoints = {0, NUM_AXES + 1}; // Second joint is invalid
        std::vector<int> modes = {VOCAB_CM_POSITION, VOCAB_CM_TORQUE};
        
        REQUIRE_FALSE(device->setControlModes(invalidJoints.size(), invalidJoints.data(), modes.data()));
        
        // Verify no changes were made
        int mode0;
        REQUIRE(device->getControlMode(0, &mode0));
        REQUIRE(mode0 == VOCAB_CM_VELOCITY);
    }
    
    SECTION("Invalid mode in subset affects entire operation")
    {
        // Set initial state
        std::vector<int> initialModes(NUM_AXES, VOCAB_CM_IDLE);
        REQUIRE(device->setControlModes(initialModes.data()));
        
        // Try to set modes with one invalid mode
        std::vector<int> joints = {0, 1};
        std::vector<int> invalidModes = {VOCAB_CM_POSITION, VOCAB_CM_POSITION_DIRECT}; // Second mode is invalid
        
        REQUIRE_FALSE(device->setControlModes(joints.size(), joints.data(), invalidModes.data()));
        
        // Verify no changes were made
        std::vector<int> unchangedModes(NUM_AXES);
        REQUIRE(device->getControlModes(unchangedModes.data()));
        for (int i = 0; i < NUM_AXES; ++i)
        {
            REQUIRE(unchangedModes[i] == VOCAB_CM_IDLE);
        }
    }
    
    SECTION("Empty subset handling")
    {
        std::vector<int> joints;
        std::vector<int> modes;
        
        // Zero-length operations should fail
        REQUIRE_FALSE(device->setControlModes(0, joints.data(), modes.data()));
        REQUIRE_FALSE(device->getControlModes(0, joints.data(), modes.data()));
        
        // Negative length should fail
        REQUIRE_FALSE(device->setControlModes(-1, joints.data(), modes.data()));
        REQUIRE_FALSE(device->getControlModes(-1, joints.data(), modes.data()));
    }
    
    SECTION("Null pointer handling in subset operations")
    {
        std::vector<int> joints = {0, 1};
        std::vector<int> modes = {VOCAB_CM_POSITION, VOCAB_CM_VELOCITY};
        
        REQUIRE_FALSE(device->setControlModes(joints.size(), nullptr, modes.data()));
        REQUIRE_FALSE(device->setControlModes(joints.size(), joints.data(), nullptr));
        REQUIRE_FALSE(device->setControlModes(joints.size(), nullptr, nullptr));
        
        REQUIRE_FALSE(device->getControlModes(joints.size(), nullptr, modes.data()));
        REQUIRE_FALSE(device->getControlModes(joints.size(), joints.data(), nullptr));
        REQUIRE_FALSE(device->getControlModes(joints.size(), nullptr, nullptr));
    }
}

TEST_CASE_METHOD(ControlModeTestFixture, "Control Mode Transitions and CiA402 Mapping", "[control_modes][transitions][cia402]")
{
    SECTION("Valid CiA402 control mode mappings")
    {
        // Based on modes_and_setpoints.md documentation:
        // POSITION -> PP (Profile Position Mode, CiA402 op 1)
        // VELOCITY -> CSV (Cyclic Synchronous Velocity Mode, CiA402 op 9)
        // TORQUE -> CST (Cyclic Synchronous Torque Mode, CiA402 op 10)  
        // CURRENT -> CST (also uses CiA402 op 10)
        // IDLE/FORCE_IDLE -> CiA402 op 0 (power stage disabled)
        
        struct ModeTest {
            int yarpMode;
            const char* modeName;
            const char* cia402Description;
        };
        
        std::vector<ModeTest> modeTests = {
            {VOCAB_CM_POSITION, "POSITION", "Profile Position (PP) - CiA402 op 1"},
            {VOCAB_CM_VELOCITY, "VELOCITY", "Cyclic Synchronous Velocity (CSV) - CiA402 op 9"},
            {VOCAB_CM_TORQUE, "TORQUE", "Cyclic Synchronous Torque (CST) - CiA402 op 10"},
            {VOCAB_CM_CURRENT, "CURRENT", "Cyclic Synchronous Torque (CST) - CiA402 op 10"},
            {VOCAB_CM_IDLE, "IDLE", "Power stage disabled - CiA402 op 0"},
            {VOCAB_CM_FORCE_IDLE, "FORCE_IDLE", "Power stage disabled/reset - CiA402 op 0"}
        };
        
        for (const auto& test : modeTests)
        {
            INFO("Testing mode: " << test.modeName << " (" << test.cia402Description << ")");
            
            for (int axis = 0; axis < NUM_AXES; ++axis)
            {
                REQUIRE(device->setControlMode(axis, test.yarpMode));
                
                int retrievedMode;
                REQUIRE(device->getControlMode(axis, &retrievedMode));
                REQUIRE(retrievedMode == test.yarpMode);
            }
        }
    }
    
    SECTION("Unsupported CiA402 modes are rejected")
    {
        // POSITION_DIRECT (CSP - Cyclic Synchronous Position) is not implemented
        // according to the documentation
        REQUIRE_FALSE(device->setControlMode(0, VOCAB_CM_POSITION_DIRECT));
        
        // Verify the mode didn't change from initial state
        int mode;
        REQUIRE(device->getControlMode(0, &mode));
        REQUIRE(mode == VOCAB_CM_IDLE); // Should still be in initial IDLE state
    }
    
    SECTION("Mode transitions between supported modes")
    {
        // Test various transitions between all supported modes
        const std::vector<int> supportedModes = {
            VOCAB_CM_POSITION,
            VOCAB_CM_VELOCITY,
            VOCAB_CM_TORQUE,
            VOCAB_CM_CURRENT,
            VOCAB_CM_IDLE,
            VOCAB_CM_FORCE_IDLE
        };
        
        for (int fromMode : supportedModes)
        {
            for (int toMode : supportedModes)
            {
                INFO("Testing transition from mode " << fromMode << " to mode " << toMode);
                
                // Set initial mode
                REQUIRE(device->setControlMode(0, fromMode));
                
                // Transition to target mode
                REQUIRE(device->setControlMode(0, toMode));
                
                // Verify transition
                int actualMode;
                REQUIRE(device->getControlMode(0, &actualMode));
                REQUIRE(actualMode == toMode);
            }
        }
    }
}

TEST_CASE_METHOD(ControlModeTestFixture, "Control Mode Edge Cases and Robustness", "[control_modes][edge_cases]")
{
    SECTION("Large scale operations")
    {
        // Test with maximum valid joint indices
        std::vector<int> allJoints(NUM_AXES);
        std::vector<int> allModes(NUM_AXES, VOCAB_CM_POSITION);
        
        for (int i = 0; i < NUM_AXES; ++i)
        {
            allJoints[i] = i;
        }
        
        REQUIRE(device->setControlModes(NUM_AXES, allJoints.data(), allModes.data()));
        
        std::vector<int> retrievedModes(NUM_AXES);
        REQUIRE(device->getControlModes(NUM_AXES, allJoints.data(), retrievedModes.data()));
        
        for (int i = 0; i < NUM_AXES; ++i)
        {
            REQUIRE(retrievedModes[i] == VOCAB_CM_POSITION);
        }
    }
    
    SECTION("Repeated operations stability")
    {
        // Repeatedly set the same mode multiple times
        for (int i = 0; i < 100; ++i)
        {
            REQUIRE(device->setControlMode(0, VOCAB_CM_VELOCITY));
            
            int mode;
            REQUIRE(device->getControlMode(0, &mode));
            REQUIRE(mode == VOCAB_CM_VELOCITY);
        }
    }
    
    SECTION("Interleaved operations consistency")
    {
        // Interleave single-joint and multi-joint operations
        REQUIRE(device->setControlMode(0, VOCAB_CM_POSITION));
        
        std::vector<int> modes = {VOCAB_CM_VELOCITY, VOCAB_CM_TORQUE, VOCAB_CM_CURRENT, VOCAB_CM_IDLE};
        REQUIRE(device->setControlModes(modes.data()));
        
        // Verify the single-joint change was overwritten by the multi-joint operation
        int mode;
        REQUIRE(device->getControlMode(0, &mode));
        REQUIRE(mode == VOCAB_CM_VELOCITY); // Should be from the multi-joint operation
    }
}
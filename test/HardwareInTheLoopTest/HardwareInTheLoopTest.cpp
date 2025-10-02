/**
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

// YARP
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/robotinterface/XMLReader.h>

// This function sets the YARP_DATA_DIRS environment variable
bool setYarpDataDirs(const std::string& installPrefix)
{
#ifdef _WIN32
    if (0 != _putenv_s("YARP_DATA_DIRS", installPrefix.c_str()))
    {
        return false;
    }
#else
    if (0 != ::setenv("YARP_DATA_DIRS", installPrefix.c_str(), true))
    {
        return false;
    }
#endif
    return true;
}

bool ensureYARPDevicesCanBeFound()
{
    // Make sure that the CiA402MotionControl device can be found
    std::string yarpDataDirs = YARP_DATA_INSTALL_DIR_FULL;
    
    // Add the build directory to find the device
    yarpDataDirs = yarpDataDirs + ":" + CMAKE_BINARY_DIR + "/share/yarp";
    
    return setYarpDataDirs(yarpDataDirs);
}

TEST_CASE("Hardware-in-the-loop test for CiA402MotionControl")
{
    // Initialize YARP network in local mode (no yarpserver needed)
    yarp::os::Network network;
    yarp::os::NetworkBase::setLocalMode(true);
    
    // Ensure YARP_DATA_DIRS contains the path to find the device
    REQUIRE(ensureYARPDevicesCanBeFound());
    
    // Get the path to the test configuration file
    std::filesystem::path configPath = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) 
                                     / std::filesystem::path("test_config.xml");
    
    INFO("Loading yarprobotinterface file from " << configPath.string());
    
    // Load the robot interface configuration
    yarp::os::Property robotInterfaceConfig;
    yarp::robotinterface::XMLReader reader;
    yarp::robotinterface::XMLReaderResult result = reader.getRobotFromFile(configPath.string(), 
                                                                           robotInterfaceConfig);
    
    REQUIRE(result.parsingIsSuccessful);
    
    // Start the device
    INFO("Starting robot interface...");
    REQUIRE(result.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));
    
    // Get the device
    REQUIRE(result.robot.hasDevice("test_joint_mc"));
    yarp::dev::PolyDriver* driver = result.robot.device("test_joint_mc").driver();
    REQUIRE(driver != nullptr);
    
    // Get the interfaces
    yarp::dev::IEncodersTimed* iEncoders = nullptr;
    yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
    yarp::dev::IPositionControl* iPositionControl = nullptr;
    yarp::dev::IVelocityControl* iVelocityControl = nullptr;
    yarp::dev::ITorqueControl* iTorqueControl = nullptr;
    yarp::dev::ICurrentControl* iCurrentControl = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IAxisInfo* iAxisInfo = nullptr;
    yarp::dev::IControlLimits* iControlLimits = nullptr;
    
    REQUIRE(driver->view(iEncoders));
    REQUIRE(driver->view(iMotorEncoders));
    REQUIRE(driver->view(iPositionControl));
    REQUIRE(driver->view(iVelocityControl));
    REQUIRE(driver->view(iTorqueControl));
    REQUIRE(driver->view(iCurrentControl));
    REQUIRE(driver->view(iControlMode));
    REQUIRE(driver->view(iAxisInfo));
    REQUIRE(driver->view(iControlLimits));
    
    REQUIRE(iEncoders != nullptr);
    REQUIRE(iMotorEncoders != nullptr);
    REQUIRE(iPositionControl != nullptr);
    REQUIRE(iVelocityControl != nullptr);
    REQUIRE(iTorqueControl != nullptr);
    REQUIRE(iCurrentControl != nullptr);
    REQUIRE(iControlMode != nullptr);
    REQUIRE(iAxisInfo != nullptr);
    REQUIRE(iControlLimits != nullptr);
    
    // Get the number of axes
    int numAxes = 0;
    REQUIRE(iEncoders->getAxes(&numAxes));
    REQUIRE(numAxes == 1);
    
    const int axisIndex = 0;
    
    SECTION("Axis info test")
    {
        std::string axisName;
        REQUIRE(iAxisInfo->getAxisName(axisIndex, axisName));
        CHECK(axisName == "joint1");
        
        yarp::dev::JointTypeEnum jointType;
        REQUIRE(iAxisInfo->getJointType(axisIndex, jointType));
    }
    
    SECTION("Control limits test")
    {
        double min = 0.0, max = 0.0;
        REQUIRE(iControlLimits->getLimits(axisIndex, &min, &max));
        INFO("Joint limits: [" << min << ", " << max << "]");
        CHECK(min < max);
    }
    
    SECTION("Encoder reading test")
    {
        // Wait a bit for the device to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Read joint encoder
        double jointPosition = 0.0;
        REQUIRE(iEncoders->getEncoder(axisIndex, &jointPosition));
        INFO("Joint position: " << jointPosition << " deg");
        
        // Read motor encoder
        double motorPosition = 0.0;
        REQUIRE(iMotorEncoders->getMotorEncoder(axisIndex, &motorPosition));
        INFO("Motor position: " << motorPosition << " deg");
        
        // Read encoder with timestamp
        double encoderTimestamp = 0.0;
        REQUIRE(iEncoders->getEncoderTimed(axisIndex, &jointPosition, &encoderTimestamp));
        CHECK(encoderTimestamp > 0.0);
        
        // Read motor encoder speed
        double motorSpeed = 0.0;
        REQUIRE(iMotorEncoders->getMotorEncoderSpeed(axisIndex, &motorSpeed));
        INFO("Motor speed: " << motorSpeed << " deg/s");
        
        // Read joint encoder speed
        double jointSpeed = 0.0;
        REQUIRE(iEncoders->getEncoderSpeed(axisIndex, &jointSpeed));
        INFO("Joint speed: " << jointSpeed << " deg/s");
    }
    
    SECTION("Control mode switching test")
    {
        // Test switching to position control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_POSITION));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        int mode = 0;
        REQUIRE(iControlMode->getControlMode(axisIndex, &mode));
        CHECK(mode == VOCAB_CM_POSITION);
        
        // Test switching to velocity control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_VELOCITY));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        REQUIRE(iControlMode->getControlMode(axisIndex, &mode));
        CHECK(mode == VOCAB_CM_VELOCITY);
        
        // Test switching to torque control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_TORQUE));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        REQUIRE(iControlMode->getControlMode(axisIndex, &mode));
        CHECK(mode == VOCAB_CM_TORQUE);
        
        // Test switching to current control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_CURRENT));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        REQUIRE(iControlMode->getControlMode(axisIndex, &mode));
        CHECK(mode == VOCAB_CM_CURRENT);
    }
    
    SECTION("Position control test")
    {
        // Switch to position control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_POSITION));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Read initial position
        double initialPosition = 0.0;
        REQUIRE(iEncoders->getEncoder(axisIndex, &initialPosition));
        INFO("Initial position: " << initialPosition << " deg");
        
        // Send a small position command (relative to current position)
        double targetPosition = initialPosition + 5.0; // Move 5 degrees
        REQUIRE(iPositionControl->positionMove(axisIndex, targetPosition));
        
        // Wait for motion to start
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Read the target position from the device
        double readTargetPosition = 0.0;
        REQUIRE(iPositionControl->getTargetPosition(axisIndex, &readTargetPosition));
        CHECK_THAT(readTargetPosition, Catch::Matchers::WithinAbs(targetPosition, 0.01));
        
        // Check if motion is done
        bool motionDone = false;
        REQUIRE(iPositionControl->checkMotionDone(axisIndex, &motionDone));
        
        // Wait for motion to complete (with timeout)
        int waitCount = 0;
        while (!motionDone && waitCount < 100)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            REQUIRE(iPositionControl->checkMotionDone(axisIndex, &motionDone));
            waitCount++;
        }
        
        // Read final position
        double finalPosition = 0.0;
        REQUIRE(iEncoders->getEncoder(axisIndex, &finalPosition));
        INFO("Final position: " << finalPosition << " deg");
        
        // Check that we moved in the right direction
        if (targetPosition > initialPosition)
        {
            CHECK(finalPosition > initialPosition);
        }
        else
        {
            CHECK(finalPosition < initialPosition);
        }
        
        // Check that we're close to the target (within a reasonable tolerance)
        // The tolerance is larger here as we're testing with actual hardware
        CHECK_THAT(finalPosition, Catch::Matchers::WithinAbs(targetPosition, 1.0));
    }
    
    SECTION("Velocity control test")
    {
        // Switch to velocity control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_VELOCITY));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Read initial position
        double initialPosition = 0.0;
        REQUIRE(iEncoders->getEncoder(axisIndex, &initialPosition));
        
        // Send a velocity command
        double velocitySetpoint = 10.0; // 10 deg/s
        REQUIRE(iVelocityControl->velocityMove(axisIndex, velocitySetpoint));
        
        // Wait for motion
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Read final position
        double finalPosition = 0.0;
        REQUIRE(iEncoders->getEncoder(axisIndex, &finalPosition));
        
        // Check that the joint moved
        CHECK(std::abs(finalPosition - initialPosition) > 0.1);
        
        // Stop the motion
        REQUIRE(iVelocityControl->velocityMove(axisIndex, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    SECTION("Torque control test")
    {
        // Switch to torque control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_TORQUE));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Read torque
        double torque = 0.0;
        REQUIRE(iTorqueControl->getTorque(axisIndex, &torque));
        INFO("Current torque: " << torque << " Nm");
        
        // Send a small torque command
        double torqueSetpoint = 0.1; // Small torque
        REQUIRE(iTorqueControl->setRefTorque(axisIndex, torqueSetpoint));
        
        // Wait a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Read torque again
        REQUIRE(iTorqueControl->getTorque(axisIndex, &torque));
        INFO("Torque after setpoint: " << torque << " Nm");
        
        // Stop the torque
        REQUIRE(iTorqueControl->setRefTorque(axisIndex, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    SECTION("Current control test")
    {
        // Switch to current control mode
        REQUIRE(iControlMode->setControlMode(axisIndex, VOCAB_CM_CURRENT));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Read current
        double current = 0.0;
        REQUIRE(iCurrentControl->getCurrent(axisIndex, &current));
        INFO("Current: " << current << " A");
        
        // Send a small current command
        double currentSetpoint = 0.1; // Small current
        REQUIRE(iCurrentControl->setRefCurrent(axisIndex, currentSetpoint));
        
        // Wait a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Read current again
        REQUIRE(iCurrentControl->getCurrent(axisIndex, &current));
        INFO("Current after setpoint: " << current << " A");
        
        // Stop the current
        REQUIRE(iCurrentControl->setRefCurrent(axisIndex, 0.0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Cleanup
    INFO("Halting robot interface...");
    REQUIRE(result.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1));
    REQUIRE(result.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown));
    
    INFO("Test completed successfully.");
}

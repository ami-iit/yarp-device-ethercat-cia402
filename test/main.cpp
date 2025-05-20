// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <atomic>
#include <csignal>
#include <memory>
#include <vector>

// -----------------------------------------------------------------------------
// Signal handling to allow clean shutdown with Ctrl+C
// -----------------------------------------------------------------------------
namespace
{
std::atomic_bool g_stopRequested{false};

void signalHandler(int signum)
{
    if (signum == SIGINT)
    {
        yInfo() << "[signalHandler] SIGINT (Ctrl+C) received. Stopping ...";
        g_stopRequested.store(true);
    }
}
} // anonymous namespace

// -----------------------------------------------------------------------------
std::shared_ptr<yarp::dev::PolyDriver>
constructDS402MotionControl(const yarp::os::Searchable& config)
{
    std::shared_ptr<yarp::dev::PolyDriver> driver;

    yarp::os::Property options;
    options.put("device", "Ds402MotionControl");

    options.put("ifname", config.find("ifname").asString());
    options.put("num_axes", config.find("num_axes").asInt32());

    driver = std::make_shared<yarp::dev::PolyDriver>();
    if (!driver->open(options))
    {
        yError() << "[constructDS402MotionControl] Failed to open the driver";
        return nullptr;
    }
    yInfo() << "[constructDS402MotionControl] Driver opened successfully";

    return driver;
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    // Register Ctrl+C handler as early as possible
    std::signal(SIGINT, signalHandler);

    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("ds402_motion_control.ini");
    rf.configure(argc, argv);

    auto driver = constructDS402MotionControl(rf);
    if (!driver)
    {
        yError() << "[main] Failed to create the driver";
        return EXIT_FAILURE;
    }

    // view the IMotorEncoders interface
    yarp::dev::IMotorEncoders* motorEncodersI = nullptr;
    if (!driver->view(motorEncodersI))
    {
        yError() << "[main] Failed to view IMotorEncoders interface";
        return EXIT_FAILURE;
    }
    yInfo() << "[main] Successfully viewed IMotorEncoders interface";

    // view the IEncodersTimed interface
    yarp::dev::IEncodersTimed* encodersTimedI = nullptr;
    if (!driver->view(encodersTimedI))
    {
        yError() << "[main] Failed to view IEncodersTimed interface";
        return EXIT_FAILURE;
    }
    yInfo() << "[main] Successfully viewed IEncodersTimed interface";

    // Determine number of axes
    int numAxes = 0;
    if (!encodersTimedI->getAxes(&numAxes) || numAxes <= 0)
    {
        yError() << "[main] Failed to retrieve the number of motor encoders or invalid value";
        return EXIT_FAILURE;
    }

    std::vector<double> motorEncoders(numAxes), motorVelocities(numAxes);
    std::vector<double> encoders(numAxes), velocities(numAxes);

    // -------------------------------------------------------------------------
    // Main loop – runs until Ctrl+C is pressed
    // -------------------------------------------------------------------------
    while (!g_stopRequested.load())
    {
        // read motor encoders
        if (!motorEncodersI->getMotorEncoders(motorEncoders.data()))
        {
            yError() << "[main] Failed to read motor encoders";
            break;
        }
        if (!motorEncodersI->getMotorEncoderSpeeds(motorVelocities.data()))
        {
            yError() << "[main] Failed to read motor encoder speeds";
            break;
        }

        // read encoders
        if (!encodersTimedI->getEncoders(encoders.data()))
        {
            yError() << "[main] Failed to read encoders";
            break;
        }
        if (!encodersTimedI->getEncoderSpeeds(velocities.data()))
        {
            yError() << "[main] Failed to read encoder speeds";
            break;
        }

        // print the values
        yInfo() << "[main] Motor encoders:" << motorEncoders << " deg | "
                << "Motor velocities:" << motorVelocities << " deg/s | "
                << "Encoders:" << encoders << " deg | "
                << "Encoder velocities:" << velocities << " deg/s";

        yarp::os::Time::delay(0.01);
    }

    // -------------------------------------------------------------------------
    // Graceful shutdown
    // -------------------------------------------------------------------------
    yInfo() << "[main] Stopping main loop and closing driver ...";
    driver->close();
    yInfo() << "[main] Driver closed successfully";

    return EXIT_SUCCESS;
}

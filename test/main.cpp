// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/dev/IMotorEncoders.h>
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
    yarp::dev::IMotorEncoders* motorEncoders = nullptr;
    if (!driver->view(motorEncoders))
    {
        yError() << "[main] Failed to view IMotorEncoders interface";
        return EXIT_FAILURE;
    }
    yInfo() << "[main] Successfully viewed IMotorEncoders interface";

    // Determine number of axes
    int numAxes = 0;
    if (!motorEncoders->getNumberOfMotorEncoders(&numAxes) || numAxes <= 0)
    {
        yError() << "[main] Failed to retrieve the number of motor encoders or invalid value";
        return EXIT_FAILURE;
    }
    std::vector<double> encoders(numAxes), velocities(numAxes);

    // -------------------------------------------------------------------------
    // Main loop – runs until Ctrl+C is pressed
    // -------------------------------------------------------------------------
    while (!g_stopRequested.load())
    {
        if (motorEncoders->getMotorEncoders(encoders.data())
            && motorEncoders->getMotorEncoderSpeeds(velocities.data()))
        {
            yInfo() << "[main] Encoder[0]:" << encoders[0] << "deg | Velocity:" << velocities[0]
                    << "deg/s";
        } else
        {
            yWarning() << "[main] Failed to read encoders";
        }

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

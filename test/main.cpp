// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <memory>
#include <vector>

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

int main(int argc, char* argv[])
{
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
    std::vector<double> encoders, velocities;
    int numAxes = 1;
    encoders.resize(numAxes);
    velocities.resize(numAxes);


    while (true)
    {
        motorEncoders->getMotorEncoders(encoders.data());
        motorEncoders->getMotorEncoderSpeeds(velocities.data());
        yInfo() << "[main] Encoder values: " << encoders[0] << "deg" << " | "
                << "Velocity: " << velocities[0] << "deg/s";
        yarp::os::Time::delay(0.01);
    }
    driver->close();
    yInfo() << "[main] Driver closed successfully";
}
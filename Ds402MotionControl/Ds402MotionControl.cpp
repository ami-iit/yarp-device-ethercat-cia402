// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Ds402MotionControl.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// SOEM
#include <ethercat.h>

#include <memory>
#include <string>
#include <string_view>

struct yarp::dev::Ds402MotionControl::Impl
{
    static constexpr std::string_view className = "Ds402MotionControl";

    Impl() = default;
    ~Impl() = default;
};

//---------------------------------------------------------------
// ctor / dtor
//---------------------------------------------------------------
yarp::dev::Ds402MotionControl::Ds402MotionControl(
    double period,
    yarp::os::ShouldUseSystemClock useSystemClock /* = yarp::os::ShouldUseSystemClock::Yes */)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
    m_impl = std::make_unique<Impl>();
}

yarp::dev::Ds402MotionControl::Ds402MotionControl()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::Yes)
{
    m_impl = std::make_unique<Impl>();
}

yarp::dev::Ds402MotionControl::~Ds402MotionControl() = default;

//---------------------------------------------------------------
// DeviceDriver
//---------------------------------------------------------------
bool yarp::dev::Ds402MotionControl::open(yarp::os::Searchable& config)
{
    // check if the parameter is present
    if (!config.check("ifname"))
    {
        yError("%s: Missing parameter 'ifname'", m_impl->className.data());
        return false;
    }
    const std::string ifname = config.find("ifname").asString();
    yInfo("Opening device %s", ifname.c_str());

    if (ec_init(ifname.c_str()) == 0)
    {
        yError("%s: Failed to initialize EtherCAT master", m_impl->className.data());
        return false;
    }

    yInfo("EtherCAT master initialized");

    return true;
}

bool yarp::dev::Ds402MotionControl::close()
{
    ec_close();
    yInfo("%s: EtherCAT master closed", m_impl->className.data());
    return true;
}

//---------------------------------------------------------------
// PeriodicThread
//---------------------------------------------------------------
void yarp::dev::Ds402MotionControl::run()
{
    return;
}

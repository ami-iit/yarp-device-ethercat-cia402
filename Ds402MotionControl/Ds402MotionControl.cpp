// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Ds402MotionControl.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// SOEM
extern "C" {
#include "ethercat.h"
}

//---------------------------------------------------------------
// ctor / dtor
//---------------------------------------------------------------
yarp::dev::Ds402MotionControl::Ds402MotionControl(double period,
                                                  yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
}

yarp::dev::Ds402MotionControl::Ds402MotionControl()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

//---------------------------------------------------------------
// DeviceDriver
//---------------------------------------------------------------
bool yarp::dev::Ds402MotionControl::open(yarp::os::Searchable& config)
{
    return true;
}

bool yarp::dev::Ds402MotionControl::close()
{

    return true;
}

//---------------------------------------------------------------
// PeriodicThread
//---------------------------------------------------------------
void yarp::dev::Ds402MotionControl::run()
{
}

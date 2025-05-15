// -*- mode:C++; c-basic-offset:4; indent-tabs-mode:nil; tab-width:4 -*-
//  Copyright (C) 2025 Istituto Italiano di Tecnologia (IIT). All rights reserved.
// This software may be modified and distributed under the terms of the
// BSD-3-Clause license.

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

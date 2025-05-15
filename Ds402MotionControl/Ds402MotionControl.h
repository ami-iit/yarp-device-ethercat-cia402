// -*- mode:C++; c-basic-offset:4; indent-tabs-mode:nil; tab-width:4 -*-
//  Copyright (C) 2025 Istituto Italiano di Tecnologia (IIT). All rights reserved.
// This software may be modified and distributed under the terms of the
// BSD-3-Clause license.

#ifndef YARP_DEV_DS402_MOTION_CONTROL_H
#define YARP_DEV_DS402_MOTION_CONTROL_H

#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Log.h>
#include <yarp/os/PeriodicThread.h>


namespace yarp
{
namespace dev
{

/**
 * @brief Minimal CiA‑402 motion‑control driver based on SOEM.
 *
 * This class owns the EtherCAT master cycle via yarp::os::PeriodicThread.
 */
class Ds402MotionControl : public DeviceDriver, public yarp::os::PeriodicThread
{
public:
    explicit Ds402MotionControl(double period,
                                yarp::os::ShouldUseSystemClock useSystemClock
                                = yarp::os::ShouldUseSystemClock::No);
    Ds402MotionControl();

    ~Ds402MotionControl() override = default;

    // ---------------- DeviceDriver ----------------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ---------------- PeriodicThread --------------
    void run() override;
};

} // namespace dev
} // namespace yarp

#endif // YARP_DEV_DS402_MOTION_CONTROL_H

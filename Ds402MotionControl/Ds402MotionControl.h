// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

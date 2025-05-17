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
    /**
     * @brief Constructor.
     *
     * @param period The period of the thread in seconds.
     * @param useSystemClock Whether to use the system clock for timing.
     */
    explicit Ds402MotionControl(double period,
                                yarp::os::ShouldUseSystemClock useSystemClock
                                = yarp::os::ShouldUseSystemClock::Yes);
    /**
     * @brief Default constructor.
     *
     * This constructor sets the period to 0.01 seconds and uses the system clock.
     */
    Ds402MotionControl();

    /**
     * @brief Destructor.
     *
     * Cleans up the resources used by the driver.
     */
    ~Ds402MotionControl() override;

    /**
     * @brief Opens the device driver.
     *
     * @param config The configuration parameters for the driver.
     * @return true if the driver was opened successfully, false otherwise.
     */
    bool open(yarp::os::Searchable& config) override;

    /**
     * @brief Closes the device driver.
     *
     * @return true if the driver was closed successfully, false otherwise.
     */
    bool close() override;

    // ---------------- PeriodicThread --------------
    /**
     * @brief Runs the periodic thread.
     *
     * This function is called periodically at the specified interval.
     */
    void run() override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace dev
} // namespace yarp

#endif // YARP_DEV_DS402_MOTION_CONTROL_H

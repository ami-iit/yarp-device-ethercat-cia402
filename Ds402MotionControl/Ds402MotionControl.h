// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_DS402_MOTION_CONTROL_H
#define YARP_DEV_DS402_MOTION_CONTROL_H

#include <memory>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IMotorEncoders.h>
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
class Ds402MotionControl : public yarp::dev::DeviceDriver,
                           public yarp::os::PeriodicThread,
                           public yarp::dev::IMotorEncoders,
                           public yarp::dev::IEncodersTimed,
                           public yarp::dev::IAxisInfo,
                           public yarp::dev::IControlMode
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

    // clang-format off
    /**
     * @brief Opens the device driver.
     *
     * @param config The configuration parameters for the driver.
     * @note The configuration parameters should include:
     * | Parameter Name         | Type     | Description                                   | Is Mandatory? |
     * |:----------------------:|:--------:|-----------------------------------------------|:-------------:|
     * | ifname                 | string   | Name of the network interface to use          | Yes           |
     * | num_axes               | int      | Number of axes to control                     | Yes           |
     * | first_slave            | int      | Index of the slave to start from (default: 1) | No            |
     * | expected_slave_name    | string   | Expected name of the slave                    | No            |
     * @return true if the driver was opened successfully, false otherwise.
     */
    bool open(yarp::os::Searchable& config) override;
    // clang-format on

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

    // ---------------- IMotorEncoders --------------

    /**
     * @brief Gets the number of motor encoders.
     * @param num Pointer to an integer where the number of encoders will be stored.
     * @return true if successful, false otherwise.
     */
    bool getNumberOfMotorEncoders(int* num) override;

    /**
     * @brief Resets the specified motor encoder to zero.
     * @param m Index of the motor encoder to reset.
     * @return true if successful, false otherwise.
     */
    bool resetMotorEncoder(int m) override;

    /**
     * @brief Resets all motor encoders to zero.
     * @return true if successful, false otherwise.
     */
    bool resetMotorEncoders() override;

    /**
     * @brief Sets the counts per revolution for a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param cpr Counts per revolution value to set.
     * @return true if successful, false otherwise.
     */
    bool setMotorEncoderCountsPerRevolution(int m, const double cpr) override;

    /**
     * @brief Gets the counts per revolution for a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param cpr Pointer to store the counts per revolution value.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderCountsPerRevolution(int m, double* cpr) override;

    /**
     * @brief Sets the value of a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param val Value to set.
     * @return true if successful, false otherwise.
     */
    bool setMotorEncoder(int m, const double val) override;

    /**
     * @brief Sets the values of all motor encoders.
     * @param vals Array of values to set for each encoder.
     * @return true if successful, false otherwise.
     */
    bool setMotorEncoders(const double* vals) override;

    /**
     * @brief Gets the value of a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param v Pointer to store the encoder value.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoder(int m, double* v) override;

    /**
     * @brief Gets the values of all motor encoders.
     * @param encs Array to store the encoder values.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoders(double* encs) override;

    /**
     * @brief Gets the values and timestamps of all motor encoders.
     * @param encs Array to store the encoder values.
     * @param time Array to store the timestamps.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncodersTimed(double* encs, double* time) override;

    /**
     * @brief Gets the value and timestamp of a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param encs Pointer to store the encoder value.
     * @param time Pointer to store the timestamp.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderTimed(int m, double* encs, double* time) override;

    /**
     * @brief Gets the speed of a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param sp Pointer to store the speed value.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderSpeed(int m, double* sp) override;

    /**
     * @brief Gets the speeds of all motor encoders.
     * @param spds Array to store the speed values.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderSpeeds(double* spds) override;

    /**
     * @brief Gets the acceleration of a specific motor encoder.
     * @param m Index of the motor encoder.
     * @param acc Pointer to store the acceleration value.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderAcceleration(int m, double* acc) override;

    /**
     * @brief Gets the accelerations of all motor encoders.
     * @param accs Array to store the acceleration values.
     * @return true if successful, false otherwise.
     */
    bool getMotorEncoderAccelerations(double* accs) override;

    // ---------------- IEncoderTimed --------------

    /**
     * @brief Gets the values and timestamps of all encoders.
     * @param encs Array to store the encoder values.
     * @param time Array to store the timestamps (in seconds).
     * @return true if successful, false otherwise.
     */
    bool getEncodersTimed(double* encs, double* time) override;

    /**
     * @brief Gets the value and timestamp of a specific encoder.
     * @param j Index of the encoder.
     * @param encs Pointer to store the encoder value.
     * @param time Pointer to store the timestamp (in seconds).
     * @return true if successful, false otherwise.
     */
    bool getEncoderTimed(int j, double* encs, double* time) override;

    /**
     * @brief Gets the number of axes (encoders).
     * @param ax Pointer to store the number of axes.
     * @return true if successful, false otherwise.
     */
    bool getAxes(int* ax) override;

    /**
     * @brief Resets the specified encoder to zero.
     * @param j Index of the encoder to reset.
     * @return true if successful, false otherwise.
     */
    bool resetEncoder(int j) override;

    /**
     * @brief Resets all encoders to zero.
     * @return true if successful, false otherwise.
     */
    bool resetEncoders() override;

    /**
     * @brief Sets the value of a specific encoder.
     * @param j Index of the encoder.
     * @param val Value to set.
     * @return true if successful, false otherwise.
     */
    bool setEncoder(int j, double val) override;

    /**
     * @brief Sets the values of all encoders.
     * @param vals Array of values to set for each encoder.
     * @return true if successful, false otherwise.
     */
    bool setEncoders(const double* vals) override;

    /**
     * @brief Gets the value of a specific encoder.
     * @param j Index of the encoder.
     * @param v Pointer to store the encoder value.
     * @return true if successful, false otherwise.
     */
    bool getEncoder(int j, double* v) override;

    /**
     * @brief Gets the values of all encoders.
     * @param encs Array to store the encoder values.
     * @return true if successful, false otherwise.
     */
    bool getEncoders(double* encs) override;

    /**
     * @brief Gets the speed of a specific encoder.
     * @param j Index of the encoder.
     * @param sp Pointer to store the speed value.
     * @return true if successful, false otherwise.
     */
    bool getEncoderSpeed(int j, double* sp) override;

    /**
     * @brief Gets the speeds of all encoders.
     * @param spds Array to store the speed values.
     * @return true if successful, false otherwise.
     */
    bool getEncoderSpeeds(double* spds) override;

    /**
     * @brief Gets the acceleration of a specific encoder.
     * @param j Index of the encoder.
     * @param spds Pointer to store the acceleration value.
     * @return true if successful, false otherwise.
     */
    bool getEncoderAcceleration(int j, double* spds) override;

    /**
     * @brief Gets the accelerations of all encoders.
     * @param accs Array to store the acceleration values.
     * @return true if successful, false otherwise.
     */
    bool getEncoderAccelerations(double* accs) override;

    // ---------------- IAxisInfo ------------------

    /**
     * @brief Gets the name of a specific axis.
     * @param axis Index of the axis.
     * @param name Reference to a string to store the axis name.
     * @return true if successful, false otherwise.
     */
    bool getAxisName(int axis, std::string& name) override;

    /**
     * @brief Gets the type of a specific axis.
     * @param axis Index of the axis.
     * @param type Reference to a JointTypeEnum to store the axis type.
     * @return true if successful, false otherwise.
     * @note For the time being, this function always returns
     * JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE.
     */
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // ---------------- IControlMode ----------------

    /**
     * @brief Gets the control mode of a specific joint.
     * @param j Index of the joint.
     * @param mode Pointer to store the control mode.
     * @return true if successful, false otherwise.
     */
    bool getControlMode(int j, int* mode) override;

    /**
     * @brief Gets the control modes of all joints.
     * @param modes Array to store the control modes.
     * @return true if successful, false otherwise.
     */
    bool getControlModes(int* modes) override;

    /**
     * @brief Gets the control modes of a subset of joints.
     * @param n Number of joints.
     * @param joints Array of joint indices.
     * @param modes Array to store the control modes.
     * @return true if successful, false otherwise.
     */
    bool getControlModes(const int n, const int* joints, int* modes) override;

    /**
     * @brief Sets the control mode of a specific joint.
     * @param j Index of the joint.
     * @param mode Control mode to set.
     * @return true if successful, false otherwise.
     */
    bool setControlMode(const int j, const int mode) override;

    /**
     * @brief Sets the control modes of a subset of joints.
     * @param n Number of joints.
     * @param joints Array of joint indices.
     * @param modes Array of control modes to set.
     * @return true if successful, false otherwise.
     */
    bool setControlModes(const int n, const int* joints, int* modes) override;

    /**
     * @brief Sets the control modes of all joints.
     * @param modes Array of control modes to set.
     * @return true if successful, false otherwise.
     */
    bool setControlModes(int* modes) override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace dev
} // namespace yarp

#endif // YARP_DEV_DS402_MOTION_CONTROL_H

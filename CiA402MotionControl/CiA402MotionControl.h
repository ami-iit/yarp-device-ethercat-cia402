// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_DEV_CIA402_MOTION_CONTROL_H
#define YARP_DEV_CIA402_MOTION_CONTROL_H

#include <memory>
#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IJointFault.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
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
class CiA402MotionControl : public yarp::dev::DeviceDriver,
                            public yarp::os::PeriodicThread,
                            public yarp::dev::IMotorEncoders,
                            public yarp::dev::IEncodersTimed,
                            public yarp::dev::IAxisInfo,
                            public yarp::dev::IControlMode,
                            public yarp::dev::ITorqueControl,
                            public yarp::dev::IVelocityControl,
                            public yarp::dev::IPositionControl,
                            public yarp::dev::IJointFault
{
public:
    /**
     * @brief Constructor.
     *
     * @param period The period of the thread in seconds.
     * @param useSystemClock Whether to use the system clock for timing.
     */
    explicit CiA402MotionControl(double period,
                                 yarp::os::ShouldUseSystemClock useSystemClock
                                 = yarp::os::ShouldUseSystemClock::Yes);
    /**
     * @brief Default constructor.
     *
     * This constructor sets the period to 0.01 seconds and uses the system clock.
     */
    CiA402MotionControl();

    /**
     * @brief Destructor.
     *
     * Cleans up the resources used by the driver.
     */
    ~CiA402MotionControl() override;

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

    // ---------------- ITorqueControl --------------

    /**
     * @brief Gets the reference torques for all joints.
     * @param t Array to store the reference torques.
     * @return true if successful, false otherwise.
     */
    bool getRefTorques(double* t) override;

    /**
     * @brief Gets the reference torque for a specific joint.
     * @param j Index of the joint.
     * @param t Pointer to store the reference torque.
     * @return true if successful, false otherwise.
     */
    bool getRefTorque(int j, double* t) override;

    /**
     * @brief Sets the reference torques for all joints.
     * @param t Array of reference torques to set.
     * @return true if successful, false otherwise.
     */
    bool setRefTorques(const double* t) override;

    /**
     * @brief Sets the reference torques for a subset of joints.
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @param t Array of reference torques to set.
     * @return true if successful, false otherwise.
     */
    bool setRefTorques(const int n_joint, const int* joints, const double* t) override;

    /**
     * @brief Sets the reference torque for a specific joint.
     * @param j Index of the joint.
     * @param t Reference torque to set.
     * @return true if successful, false otherwise.
     */
    bool setRefTorque(int j, double t) override;

    /**
     * @brief Gets the measured torque for a specific joint.
     * @param j Index of the joint.
     * @param t Pointer to store the measured torque.
     * @return true if successful, false otherwise.
     */
    bool getTorque(int j, double* t) override;

    /**
     * @brief Gets the measured torques for all joints.
     * @param t Array to store the measured torques.
     * @return true if successful, false otherwise.
     */
    bool getTorques(double* t) override;

    /**
     * @brief Gets the torque range for a specific joint.
     * @param j Index of the joint.
     * @param min Pointer to store the minimum torque.
     * @param max Pointer to store the maximum torque.
     * @return true if successful, false otherwise.
     */
    bool getTorqueRange(int j, double* min, double* max) override;

    /**
     * @brief Gets the torque ranges for all joints.
     * @param min Array to store the minimum torques.
     * @param max Array to store the maximum torques.
     * @return true if successful, false otherwise.
     */
    bool getTorqueRanges(double* min, double* max) override;

    // ---------------- IVelocityControl --------------

    /**
     * @brief Commands a velocity move for a specific joint.
     *
     * @param j Index of the joint.
     * @param sp Desired velocity setpoint (units: deg/s or rad/s, depending on configuration).
     * @return true if successful, false otherwise.
     *
     * @note This interface implements Cyclic Synchronous Velocity Mode (CSV), so the command is
     * sent cyclically.
     */
    bool velocityMove(int j, double sp) override;

    /**
     * @brief Commands velocity moves for all joints.
     *
     * @param sp Array of desired velocity setpoints for each joint.
     * @return true if successful, false otherwise.
     *
     * @note This interface implements Cyclic Synchronous Velocity Mode (CSV), so the command is
     * sent cyclically.
     */
    bool velocityMove(const double* sp) override;

    /**
     * @brief Gets the reference velocity for a specific joint.
     *
     * @param joint Index of the joint.
     * @param vel Pointer to store the reference velocity.
     * @return true if successful, false otherwise.
     */
    bool getRefVelocity(const int joint, double* vel) override;

    /**
     * @brief Gets the reference velocities for all joints.
     *
     * @param vels Array to store the reference velocities.
     * @return true if successful, false otherwise.
     */
    bool getRefVelocities(double* vels) override;

    /**
     * @brief Gets the reference velocity for a subset of joints.
     *
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @param vels Array to store the reference velocities.
     * @return true if successful, false otherwise.
     */
    bool getRefVelocities(const int n_joint, const int* joints, double* vels) override;

    /**
     * @brief (Unused in CSV mode) Sets the reference acceleration for a specific joint.
     *
     * @param j Index of the joint.
     * @param acc Reference acceleration (ignored).
     * @return Always returns false. Not used in Cyclic Synchronous Velocity Mode.
     */
    bool setRefAcceleration(int j, double acc) override;

    /**
     * @brief (Unused in CSV mode) Sets the reference accelerations for all joints.
     *
     * @param accs Array of reference accelerations (ignored).
     * @return Always returns false. Not used in Cyclic Synchronous Velocity Mode.
     */
    bool setRefAccelerations(const double* accs) override;

    /**
     * @brief (Unused in CSV mode) Gets the reference acceleration for a specific joint.
     *
     * @param j Index of the joint.
     * @param acc Pointer to store the reference acceleration (ignored).
     * @return Always returns false. Not used in Cyclic Synchronous Velocity Mode.
     */
    bool getRefAcceleration(int j, double* acc) override;

    /**
     * @brief (Unused in CSV mode) Gets the reference accelerations for all joints.
     *
     * @param accs Array to store the reference accelerations (ignored).
     * @return Always returns false. Not used in Cyclic Synchronous Velocity Mode.
     */
    bool getRefAccelerations(double* accs) override;

    /**
     * @brief Stops motion for a specific joint.
     *
     * @param j Index of the joint.
     * @return true if successful, false otherwise.
     *
     * @note In CSV mode, this typically sets the velocity setpoint to zero for the specified joint.
     */
    bool stop(int j) override;

    /**
     * @brief Stops motion for all joints.
     *
     * @return true if successful, false otherwise.
     *
     * @note In CSV mode, this typically sets all velocity setpoints to zero.
     */
    bool stop() override;

    /**
     * @brief Commands velocity moves for a subset of joints.
     *
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @param spds Array of desired velocity setpoints for the specified joints.
     * @return true if successful, false otherwise.
     *
     * @note This interface implements Cyclic Synchronous Velocity Mode (CSV), so the command is
     * sent cyclically.
     */
    bool velocityMove(const int n_joint, const int* joints, const double* spds) override;

    /**
     * @brief Sets the reference accelerations for a subset of joints.
     *
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @param accs Array of reference accelerations (ignored).
     * @note This function is not used in Cyclic Synchronous Velocity Mode (CSV).
     * @note If the driver is switched to Profile Position Mode (PP), this function call a SDO write
     * and set the reference acceleration (0x6083).
     *
     * @return True in case of success, false otherwise.
     */
    bool setRefAccelerations(const int n_joint, const int* joints, const double* accs) override;

    /**
     * @brief Gets the reference accelerations for a subset of joints.
     *
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @param accs Array to store the reference accelerations (ignored).
     * @note This function is not used in Cyclic Synchronous Velocity Mode (CSV).
     * @note If the driver is switched to Profile Position Mode (PP), this function call a SDO read
     * and get the reference acceleration (0x6083).
     * @return True in case of success, false otherwise.
     */
    bool getRefAccelerations(const int n_joint, const int* joints, double* accs) override;

    /**
     * @brief Stops motion for a subset of joints.
     *
     * @param n_joint Number of joints.
     * @param joints Array of joint indices.
     * @return true if successful, false otherwise.
     *
     * @note In CSV mode, this typically sets the velocity setpoints to zero for the specified
     * joints.
     * @note If the driver is switched to Profile Position Mode (PP), this function call a SDO write
     * and set the "Halt Position" command (0x6040, 0x0006) for each specified joint.
     */
    bool stop(const int n_joint, const int* joints) override;

    // ---------------- IJointFault ----------------
    /**
     * @brief Gets the last joint fault for a specific joint.
     * @param j Index of the joint.
     * @param fault Reference to an integer to store the fault code.
     * @param message Reference to a string to store the fault message.
     * @return true if successful, false otherwise.
     */
    bool getLastJointFault(int j, int& fault, std::string& message) override;

    // ---------------- IPositionControl --------------

    /**
     * @brief Commands an absolute position move for a specific joint using Profile Position Mode
     * (PP).
     *
     * This function initiates a trapezoidal position trajectory from the current position
     * to the specified target position. The motion profile is governed by the configured
     * profile velocity (0x6081) and profile acceleration (0x6083).
     *
     * @param j Index of the joint to move (0-based).
     * @param ref Target position in joint coordinates (degrees or radians, depending on
     * configuration).
     * @return true if the command was successfully sent, false otherwise.
     *
     * @note This function uses CiA-402 Profile Position Mode. The actual motion execution
     *       depends on the drive's state machine being in "Operation Enabled" state.
     * @note The motion is non-blocking. Use checkMotionDone() to verify completion.
     * @note Position units depend on the encoder configuration and gear ratios.
     */
    bool positionMove(int j, double ref) override;

    /**
     * @brief Commands absolute position moves for all joints simultaneously.
     *
     * This function initiates coordinated trapezoidal position trajectories for all joints.
     * Each joint moves from its current position to the corresponding target position in
     * the refs array using individual profile parameters.
     *
     * @param refs Array of target positions for all joints (size must equal number of axes).
     * @return true if all commands were successfully sent, false otherwise.
     *
     * @note Joints move simultaneously but independently - no trajectory synchronization.
     * @note Array size must match the number of configured axes.
     * @note Use checkMotionDone() to verify when all motions are complete.
     */
    bool positionMove(const double* refs) override;

    /**
     * @brief Commands absolute position moves for a subset of joints.
     *
     * This function allows selective position control of specific joints while leaving
     * others unaffected. Each specified joint moves to its target position using
     * Profile Position Mode.
     *
     * @param n_joint Number of joints to move (must be > 0).
     * @param joints Array of joint indices to move (each must be valid joint index).
     * @param refs Array of target positions corresponding to the specified joints.
     * @return true if all commands were successfully sent, false otherwise.
     *
     * @note joints and refs arrays must have the same size (n_joint elements).
     * @note Non-specified joints remain in their current control state.
     * @note Joint indices must be valid (0 to num_axes-1).
     */
    bool positionMove(const int n_joint, const int* joints, const double* refs) override;

    /**
     * @brief Commands a relative position move for a specific joint.
     *
     * This function moves the joint by the specified delta amount from its current
     * position. The motion uses Profile Position Mode with a calculated target
     * position = current_position + delta.
     *
     * @param j Index of the joint to move (0-based).
     * @param delta Relative displacement in joint coordinates (degrees or radians).
     * @return true if the command was successfully sent, false otherwise.
     *
     * @note The current position is read at command time to calculate the absolute target.
     * @note Positive delta values move in the positive joint direction.
     * @note Motion profile follows the same trapezoidal trajectory as absolute moves.
     */
    bool relativeMove(int j, double delta) override;

    /**
     * @brief Commands relative position moves for all joints simultaneously.
     *
     * This function moves all joints by their respective delta amounts from their
     * current positions. Each joint's target is calculated as current_position + delta.
     *
     * @param deltas Array of relative displacements for all joints (size must equal number of
     * axes).
     * @return true if all commands were successfully sent, false otherwise.
     *
     * @note Current positions are sampled at command time for target calculation.
     * @note All joints move simultaneously but independently.
     * @note Zero delta values result in no motion for those joints.
     */
    bool relativeMove(const double* deltas) override;

    /**
     * @brief Commands relative position moves for a subset of joints.
     *
     * This function moves specific joints by their respective delta amounts while
     * leaving other joints unaffected. Targets are calculated from current positions.
     *
     * @param n_joint Number of joints to move (must be > 0).
     * @param joints Array of joint indices to move (each must be valid joint index).
     * @param deltas Array of relative displacements corresponding to the specified joints.
     * @return true if all commands were successfully sent, false otherwise.
     *
     * @note joints and deltas arrays must have the same size (n_joint elements).
     * @note Current positions are read for target calculation at command time.
     * @note Non-specified joints are not affected by this command.
     */
    bool relativeMove(const int n_joint, const int* joints, const double* deltas) override;

    /**
     * @brief Checks if a specific joint has completed its motion.
     *
     * This function queries the CiA-402 Status Word (0x6041) to determine if the
     * joint has reached its target position. The check includes both position
     * tolerance and velocity settling criteria.
     *
     * @param j Index of the joint to check (0-based).
     * @param flag Pointer to store the motion status (true if motion complete, false if still
     * moving).
     * @return true if the status was successfully retrieved, false on error.
     *
     * @note Motion is considered complete when the drive reports "Target Reached" status.
     * @note This includes both position accuracy and velocity settling requirements.
     * @note The function checks the drive's internal motion status, not just position error.
     */
    bool checkMotionDone(int j, bool* flag) override;

    /**
     * @brief Checks if all joints have completed their motions.
     *
     * This function checks the motion completion status for all configured joints
     * and returns true only when all joints have reached their targets.
     *
     * @param flag Pointer to store the overall motion status (true if all motions complete).
     * @return true if the status check was successful, false on error.
     *
     * @note Returns true in flag only when ALL joints report "Target Reached".
     * @note Useful for coordinated motion sequences where all axes must complete.
     * @note Individual joint errors will cause this function to return false.
     */
    bool checkMotionDone(bool* flag) override;

    /**
     * @brief Checks if specific joints have completed their motions.
     *
     * This function checks motion completion status for a subset of joints,
     * returning true only when all specified joints have reached their targets.
     *
     * @param n_joint Number of joints to check (must be > 0).
     * @param joints Array of joint indices to check (each must be valid joint index).
     * @param flag Pointer to store the motion status (true if all specified motions complete).
     * @return true if the status check was successful, false on error.
     *
     * @note Returns true in flag only when ALL specified joints report "Target Reached".
     * @note Non-specified joints are ignored in the completion check.
     * @note Joint indices must be valid (0 to num_axes-1).
     */
    bool checkMotionDone(const int n_joint, const int* joints, bool* flag) override;

    /**
     * @brief Sets the reference speed (profile velocity) for a specific joint.
     *
     * This function configures the maximum velocity used during position moves
     * by writing to CiA-402 object 0x6081 (Profile Velocity) via SDO communication.
     * This speed affects all subsequent position moves for the joint.
     *
     * @param j Index of the joint (0-based).
     * @param sp Profile velocity in joint coordinates per second (deg/s or rad/s).
     * @return true if the parameter was successfully written, false otherwise.
     *
     * @note This sets the maximum velocity for trapezoidal motion profiles.
     * @note The actual motion speed may be lower due to acceleration limits.
     * @note Changes take effect for subsequent position commands, not current motions.
     * @note SDO communication may introduce latency compared to PDO-based commands.
     */
    bool setRefSpeed(int j, double sp) override;

    /**
     * @brief Sets the reference speeds (profile velocities) for all joints.
     *
     * This function configures the profile velocities for all joints simultaneously
     * by writing to each joint's 0x6081 object via SDO communication.
     *
     * @param spds Array of profile velocities for all joints (size must equal number of axes).
     * @return true if all parameters were successfully written, false otherwise.
     *
     * @note Array size must match the number of configured axes.
     * @note All SDO writes are performed sequentially, which may take time.
     * @note Settings apply to subsequent position moves, not current motions.
     * @note If any single joint fails, the entire operation may be marked as failed.
     */
    bool setRefSpeeds(const double* spds) override;

    /**
     * @brief Sets the reference speeds (profile velocities) for specific joints.
     *
     * This function allows selective configuration of profile velocities for
     * a subset of joints while leaving others unchanged.
     *
     * @param n_joint Number of joints to configure (must be > 0).
     * @param joints Array of joint indices to configure (each must be valid joint index).
     * @param spds Array of profile velocities corresponding to the specified joints.
     * @return true if all parameters were successfully written, false otherwise.
     *
     * @note joints and spds arrays must have the same size (n_joint elements).
     * @note SDO writes are performed only for specified joints.
     * @note Non-specified joints retain their current profile velocity settings.
     * @note Joint indices must be valid (0 to num_axes-1).
     */
    bool setRefSpeeds(const int n_joint, const int* joints, const double* spds) override;

    /**
     * @brief Gets the current reference speed (profile velocity) for a specific joint.
     *
     * This function reads the currently configured profile velocity from
     * CiA-402 object 0x6081 via SDO communication.
     *
     * @param j Index of the joint (0-based).
     * @param ref Pointer to store the current profile velocity value.
     * @return true if the parameter was successfully read, false otherwise.
     *
     * @note This reads the configured maximum velocity for position profiles.
     * @note SDO communication may introduce latency for real-time applications.
     * @note The returned value reflects the drive's internal configuration.
     */
    bool getRefSpeed(int j, double* ref) override;

    /**
     * @brief Gets the current reference speeds (profile velocities) for all joints.
     *
     * This function reads the configured profile velocities from all joints
     * by querying each joint's 0x6081 object via SDO communication.
     *
     * @param spds Array to store the profile velocities for all joints (size must equal number of
     * axes).
     * @return true if all parameters were successfully read, false otherwise.
     *
     * @note Array size must match the number of configured axes.
     * @note All SDO reads are performed sequentially, which may take time.
     * @note If any single joint read fails, the entire operation may fail.
     * @note Values reflect the drives' current internal configuration.
     */
    bool getRefSpeeds(double* spds) override;

    /**
     * @brief Gets the current reference speeds (profile velocities) for specific joints.
     *
     * This function reads the profile velocity configuration for a subset of
     * joints, allowing selective monitoring of profile parameters.
     *
     * @param n_joint Number of joints to query (must be > 0).
     * @param joints Array of joint indices to query (each must be valid joint index).
     * @param spds Array to store the profile velocities corresponding to the specified joints.
     * @return true if all parameters were successfully read, false otherwise.
     *
     * @note joints and spds arrays must have the same size (n_joint elements).
     * @note SDO reads are performed only for specified joints.
     * @note Joint indices must be valid (0 to num_axes-1).
     * @note Values reflect the drives' current profile velocity settings.
     */
    bool getRefSpeeds(const int n_joint, const int* joints, double* spds) override;

    /**
     * @brief Gets the target position for a specific joint.
     *
     * This function reads the target position that was last commanded
     * to the specified joint. The value reflects the last positionMove()
     * or relativeMove() command issued.
     *
     * @param j Index of the joint (0-based).
     * @param ref Pointer to store the target position value.
     * @return true if the parameter was successfully read, false otherwise.
     *
     * @note The target position is in joint coordinates (degrees or radians).
     * @note If no position command has been issued, the value may be undefined.
     * @note This function does not read the actual current position.
     */
    bool getTargetPosition(int j, double* ref) override;

    /**
     * @brief Gets the target positions for all joints.
     *
     * This function reads the target positions that were last commanded
     * to all joints. The values reflect the last positionMove() or
     * relativeMove() commands issued.
     *
     * @param refs Array to store the target positions for all joints (size must equal number of
     * axes).
     * @return true if all parameters were successfully read, false otherwise.
     *
     * @note Target positions are in joint coordinates (degrees or radians).
     * @note If no position commands have been issued, values may be undefined.
     * @note This function does not read the actual current positions.
     */
    bool getTargetPositions(double* refs) override;

    /**
     * @brief Gets the target positions for specific joints.
     *
     * This function reads the target positions that were last commanded
     * to a subset of joints. The values reflect the last positionMove()
     * or relativeMove() commands issued for those joints.
     *
     * @param n_joint Number of joints to query (must be > 0).
     * @param joints Array of joint indices to query (each must be valid joint index).
     * @param refs Array to store the target positions corresponding to the specified joints.
     * @return true if all parameters were successfully read, false otherwise.
     *
     * @note joints and refs arrays must have the same size (n_joint elements).
     * @note Target positions are in joint coordinates (degrees or radians).
     * @note If no position commands have been issued for a joint, its value may be undefined.
     * @note This function does not read the actual current positions.
     */
    bool getTargetPositions(const int n_joint, const int* joints, double* refs) override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace dev
} // namespace yarp

#endif // YARP_DEV_CIA402_MOTION_CONTROL_H

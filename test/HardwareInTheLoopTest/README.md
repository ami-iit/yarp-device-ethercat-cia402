# Hardware-in-the-Loop Test for CiA402MotionControl

This test validates the CiA402MotionControl device with actual hardware connected via EtherCAT.

## Prerequisites

1. **Hardware**: A single-joint EtherCAT ring with a CiA402-compliant drive must be connected to the test machine.
2. **Network Interface**: The EtherCAT network interface must be available (default: `eth0`).
3. **Permissions**: The test executable needs network capabilities to access the EtherCAT interface.
4. **YARP Server**: No YARP server is needed (the test uses YARP in local mode).

## Building the Test

To enable and build the test:

```bash
cd build
cmake .. -DBUILD_TESTING=ON
cmake --build .
```

## Running the Test

### 1. Set up network interface permissions

The test needs to access the EtherCAT network interface. You can either:

**Option A**: Run as root (not recommended)
```bash
sudo ./test/HardwareInTheLoopTest/HardwareInTheLoopTest
```

**Option B**: Set capabilities on the test executable (recommended)
```bash
sudo setcap cap_net_raw,cap_net_admin+ep ./test/HardwareInTheLoopTest/HardwareInTheLoopTest
./test/HardwareInTheLoopTest/HardwareInTheLoopTest
```

### 2. Configure the network interface

Edit `test/HardwareInTheLoopTest/test_config.xml` to set the correct network interface name:

```xml
<param name="ifname">"eth0"</param>  <!-- Change eth0 to your interface -->
```

### 3. Run the test

```bash
cd build
./test/HardwareInTheLoopTest/HardwareInTheLoopTest
```

Or using CTest:
```bash
cd build
ctest -L hardware -V
```

## Test Coverage

The test validates the following functionality:

1. **Device Loading**: Loads the CiA402MotionControl device via yarprobotinterface
2. **Interface Availability**: Verifies all control interfaces are available
3. **Axis Information**: Reads axis names and joint types
4. **Control Limits**: Reads and validates joint limits
5. **Encoder Reading**: Tests joint and motor encoder reading with timestamps
6. **Control Mode Switching**: Tests switching between different control modes:
   - Position control (VOCAB_CM_POSITION)
   - Velocity control (VOCAB_CM_VELOCITY)
   - Torque control (VOCAB_CM_TORQUE)
   - Current control (VOCAB_CM_CURRENT)
7. **Position Control**: Sends position setpoints and validates motion
8. **Velocity Control**: Sends velocity commands and validates motion
9. **Torque Control**: Sends torque setpoints and reads torque feedback
10. **Current Control**: Sends current setpoints and reads current feedback

## Test Sections

The test uses Catch2 sections to organize different test cases:
- `Axis info test`
- `Control limits test`
- `Encoder reading test`
- `Control mode switching test`
- `Position control test`
- `Velocity control test`
- `Torque control test`
- `Current control test`

You can run specific sections using Catch2's command-line options:
```bash
./test/HardwareInTheLoopTest/HardwareInTheLoopTest "[Position control test]"
```

## Troubleshooting

### "Cannot initialize on eth0"
- Check that the network interface exists: `ip link show`
- Verify the interface name in `test_config.xml`
- Check that no other process is using the EtherCAT interface

### "No slaves found"
- Verify that the EtherCAT hardware is powered on
- Check cable connections
- Verify that the drive is in a ready state

### "Timeout waiting for motion"
- The drive may have limits or safety features preventing motion
- Check drive configuration and error logs
- Verify that the commanded positions are within limits

### Permission denied
- Run with sudo or set capabilities: `sudo setcap cap_net_raw,cap_net_admin+ep <executable>`

## Notes

- This test is designed for **manual execution** with hardware connected
- The test should **not** be run in CI/CD pipelines without hardware
- The test uses YARP in local mode, so no yarpserver is needed
- Test timeouts are set to 60 seconds to allow for hardware initialization
- The test is marked with labels "hardware" and "manual" for filtering

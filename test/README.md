# CiA402MotionControl Test Suite

This directory contains a comprehensive Catch2 test suite for the CiA402MotionControl device, specifically focused on testing control modes functionality.

## Overview

The test suite provides comprehensive coverage of the IControlMode interface implementation, testing various scenarios including:

- Basic control mode operations (get/set)
- Control mode validation and CiA402 mapping  
- Error handling and edge cases
- Multi-joint operations
- Mode transitions
- Robustness and reliability

## Test Structure

### Files

- `unit_tests/test_control_modes.cpp` - Main test suite with comprehensive control mode tests
- `unit_tests/test_main.cpp` - Catch2 main entry point
- `unit_tests/mock_yarp_vocabs.h` - Mock YARP vocabulary constants for testing
- `catch2/` - Catch2 framework (amalgamated version)
- `main.cpp` - Original integration test (requires YARP and hardware)

### Test Categories

#### 1. Control Mode Basic Functionality `[control_modes][basic]`
- Device initialization verification
- Initial state validation (all axes should start in IDLE mode)
- Basic get/set operations

#### 2. Single Joint Control Mode Operations `[control_modes][single_joint]`  
- Individual joint mode setting/getting
- Supported mode validation:
  - `VOCAB_CM_POSITION` - Profile Position (PP) mode
  - `VOCAB_CM_VELOCITY` - Cyclic Synchronous Velocity (CSV) mode
  - `VOCAB_CM_TORQUE` - Cyclic Synchronous Torque (CST) mode
  - `VOCAB_CM_CURRENT` - Current control (also uses CST mode)
  - `VOCAB_CM_IDLE` - Power stage disabled
  - `VOCAB_CM_FORCE_IDLE` - Force idle/reset
- Invalid joint index handling
- Unsupported mode rejection (`VOCAB_CM_POSITION_DIRECT` is not implemented)
- Null pointer validation

#### 3. Multiple Joints Control Mode Operations `[control_modes][multiple_joints]`
- Setting control modes for all joints simultaneously  
- Atomic operation validation (all-or-nothing semantics)
- Error propagation handling
- Input validation

#### 4. Subset of Joints Control Mode Operations `[control_modes][subset]`
- Setting control modes for specific subsets of joints
- Partial update operations
- Array validation and bounds checking
- Mixed valid/invalid input handling

#### 5. Control Mode Transitions and CiA402 Mapping `[control_modes][transitions][cia402]`
- CiA402 operation mode mapping verification:
  - `POSITION` → PP (Profile Position Mode, CiA402 op 1)
  - `VELOCITY` → CSV (Cyclic Synchronous Velocity Mode, CiA402 op 9) 
  - `TORQUE` → CST (Cyclic Synchronous Torque Mode, CiA402 op 10)
  - `CURRENT` → CST (Cyclic Synchronous Torque Mode, CiA402 op 10)
  - `IDLE`/`FORCE_IDLE` → CiA402 op 0 (power stage disabled)
- Mode transition validation between all supported modes
- Unsupported mode rejection (`POSITION_DIRECT` - CSP not implemented)

#### 6. Control Mode Edge Cases and Robustness `[control_modes][edge_cases]`
- Large scale operations (all joints simultaneously)
- Repeated operations stability  
- Interleaved operation consistency
- Performance and reliability validation

## Building and Running Tests

### Prerequisites

The unit tests are designed to run without requiring YARP installation:

- CMake 3.8+
- C++17 compatible compiler
- Catch2 v3.5.4 (included as amalgamated files)

### Build Instructions

#### Option 1: Standalone Test Build (Recommended)
```bash
cd test/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4
```

#### Option 2: Full Project Build (Requires YARP)
```bash
mkdir build && cd build  
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4
```

### Running Tests

#### Run All Tests
```bash
./CiA402MotionControlUnitTests
```

#### Run Specific Test Categories
```bash
# Run only basic functionality tests
./CiA402MotionControlUnitTests [basic]

# Run only single joint tests  
./CiA402MotionControlUnitTests [single_joint]

# Run only CiA402 mapping tests
./CiA402MotionControlUnitTests [cia402] 

# Run only edge case tests
./CiA402MotionControlUnitTests [edge_cases]
```

#### Run with Detailed Output
```bash
./CiA402MotionControlUnitTests --success
```

#### Using CTest
```bash
make test
# or
ctest -V
```

## Mock Implementation

The test suite uses a `MockControlModeDevice` class that implements the `IControlMode` interface without requiring actual EtherCAT hardware. This mock:

- Simulates the behavior of CiA402MotionControl control mode functionality
- Validates input parameters and mode values
- Maintains internal state for testing
- Provides the same interface contracts as the real device
- Enforces CiA402 mode restrictions (e.g., POSITION_DIRECT is not supported)

## Test Coverage

The test suite provides comprehensive coverage of:

✅ **Interface Compliance**
- All IControlMode interface methods
- Parameter validation  
- Return value correctness
- Error condition handling

✅ **CiA402 Specification Compliance**  
- Supported operation modes (PP, CSV, CST, IDLE)
- Unsupported mode rejection (CSP - POSITION_DIRECT)
- Mode mapping according to specification

✅ **Robustness**
- Invalid input handling
- Boundary conditions
- Null pointer safety
- Large scale operations
- Repeated operations
- Concurrent operation scenarios

✅ **Functional Requirements**
- Mode setting/getting for individual joints
- Mode setting/getting for multiple joints 
- Mode setting/getting for joint subsets
- Atomic operation semantics
- State consistency

## Integration with CI/CD

The test suite is designed to integrate seamlessly with continuous integration:

- **No Hardware Dependencies**: Tests run using mocks
- **Fast Execution**: All tests complete in under 1 second
- **Clear Output**: Detailed test results with assertion counts
- **Exit Codes**: Proper exit codes for CI/CD integration (0 = success, non-zero = failure)

## Extending the Test Suite

To add new tests:

1. **Add test cases** in `test_control_modes.cpp` following the existing pattern
2. **Use appropriate tags** to categorize tests (e.g., `[control_modes][new_feature]`)
3. **Follow naming conventions** for test sections and descriptions
4. **Add documentation** for new test categories in this README
5. **Ensure mock compatibility** - update MockControlModeDevice if needed

### Example Test Case Template

```cpp
TEST_CASE_METHOD(ControlModeTestFixture, "New Feature Test", "[control_modes][new_feature]")
{
    SECTION("Test scenario description")
    {
        // Arrange
        // ... setup test data
        
        // Act  
        // ... call device methods
        
        // Assert
        REQUIRE(/* condition */);
        
        // Additional validations
        INFO("Descriptive message for debugging");
        REQUIRE(/* another condition */);
    }
}
```

## Known Limitations

- **Mock-based Testing**: Tests use mocks rather than real hardware, so hardware-specific issues may not be caught
- **Interface Focus**: Currently focuses primarily on IControlMode interface
- **CiA402 Simulation**: Mock behavior is based on documentation rather than actual CiA402 drive behavior

## Future Enhancements

Potential areas for expansion:

- [ ] Additional interface testing (IPositionControl, IVelocityControl, etc.)
- [ ] Hardware-in-the-loop testing integration  
- [ ] Performance benchmarking
- [ ] Stress testing with high-frequency operations
- [ ] Integration tests with actual CiA402 drives (when hardware available)
- [ ] State machine transition testing
- [ ] Error recovery scenario testing
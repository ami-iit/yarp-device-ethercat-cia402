cmake_minimum_required(VERSION 3.8)

# Integration test application (original test) - requires YARP
add_application(
    NAME CiA402MotionControlTest
    SOURCES main.cpp
    LINK_LIBRARIES  ${YARP_LIBRARIES})

# Catch2 unit tests - can build without YARP installation
add_library(Catch2 STATIC 
    catch2/catch_amalgamated.cpp
)
target_include_directories(Catch2 PUBLIC catch2)
target_compile_features(Catch2 PUBLIC cxx_std_17)

# Unit test executable - uses mocks, doesn't require YARP
add_executable(CiA402MotionControlUnitTests
    unit_tests/test_control_modes.cpp
    unit_tests/test_main.cpp
)

target_link_libraries(CiA402MotionControlUnitTests 
    PRIVATE 
    Catch2
)

target_include_directories(CiA402MotionControlUnitTests 
    PRIVATE 
    ../CiA402MotionControl
    catch2
    unit_tests
)

target_compile_features(CiA402MotionControlUnitTests PRIVATE cxx_std_17)

# Enable testing
enable_testing()
add_test(NAME CiA402_ControlModes COMMAND CiA402MotionControlUnitTests)
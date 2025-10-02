# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Find Catch2
find_package(Catch2 3 QUIET)

if(NOT Catch2_FOUND)
    message(STATUS "Catch2 not found, trying to fetch it")
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.2
    )
    FetchContent_MakeAvailable(Catch2)
endif()

# Find YARP robotinterface
find_package(YARP_robotinterface REQUIRED)

# Get YARP data install directory
get_property(YARP_DATA_INSTALL_DIR TARGET YARP::YARP_os PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
list(GET YARP_DATA_INSTALL_DIR 0 YARP_DATA_INSTALL_DIR_FULL)
get_filename_component(YARP_DATA_INSTALL_DIR_FULL "${YARP_DATA_INSTALL_DIR_FULL}" DIRECTORY)
get_filename_component(YARP_DATA_INSTALL_DIR_FULL "${YARP_DATA_INSTALL_DIR_FULL}" DIRECTORY)

# Create the test executable
add_executable(HardwareInTheLoopTest HardwareInTheLoopTest.cpp)

target_link_libraries(HardwareInTheLoopTest 
    PRIVATE 
    Catch2::Catch2WithMain
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_robotinterface
)

target_compile_definitions(HardwareInTheLoopTest 
    PRIVATE 
    CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    CMAKE_BINARY_DIR="${CMAKE_BINARY_DIR}"
    YARP_DATA_INSTALL_DIR_FULL="${YARP_DATA_INSTALL_DIR_FULL}"
)

# Note: This test requires hardware to be connected
# It should be run manually, not in CI
add_test(NAME HardwareInTheLoopTest 
         COMMAND HardwareInTheLoopTest
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Set test properties
set_tests_properties(HardwareInTheLoopTest PROPERTIES
    LABELS "hardware;manual"
    TIMEOUT 60
)

# Install the test configuration file
install(FILES test_config.xml 
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/tests/HardwareInTheLoopTest)

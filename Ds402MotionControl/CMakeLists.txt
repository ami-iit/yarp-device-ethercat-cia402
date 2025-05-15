# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


add_yarp_device(
    NAME Ds402MotionControl
    TYPE yarp::dev::Ds402MotionControl
    SOURCES Ds402MotionControl.cpp
    PUBLIC_HEADERS Ds402MotionControl.h
    PUBLIC_LINK_LIBRARIES
      YARP::YARP_os
      YARP::YARP_dev
      soem
    CONFIGURE_PACKAGE_NAME ds402_motion_control)

# yarp_prepare_plugin(ds402 TYPE yarp::dev::ds402
#                             INCLUDE Ds402MotionControl.h
#                             CATEGORY device
#                             DEFAULT ON)


# set(PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Ds402MotionControl.cpp")
# set(PLUGIN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/Ds402MotionControl.h")

# yarp_add_plugin(ds402 ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

# target_link_libraries(ds402 YARP::YARP_OS YARP::YARP_dev YARP::YARP_sig soem)

# target_include_directories(ds402 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
  
# yarp_install(TARGETS ds402
#             COMPONENT runtime
#             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
#             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})
         
# yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/ds402.ini"  DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})



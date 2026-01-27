# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


add_yarp_device(
    NAME CiA402MotionControl
    TYPE yarp::dev::CiA402MotionControl
    SOURCES CiA402MotionControl.cpp CiA402StateMachine.cpp
    PUBLIC_HEADERS CiA402MotionControl.h CiA402StateMachine.h
    PUBLIC_LINK_LIBRARIES
      YARP::YARP_os
      YARP::YARP_dev
      YarpCiA402::EthercatManager
    CONFIGURE_PACKAGE_NAME CiA402MotionControlPlugins)

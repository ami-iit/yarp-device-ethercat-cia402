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
    CONFIGURE_PACKAGE_NAME Ds402MotionControl)

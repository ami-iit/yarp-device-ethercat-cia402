// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file mock_yarp_vocabs.h
 * @brief Mock YARP vocabulary constants for unit testing
 * 
 * This header provides the minimal set of YARP vocabulary constants needed
 * for testing CiA402MotionControl control modes without requiring a full
 * YARP installation.
 */

#pragma once

// Mock YARP control mode vocabulary constants
// These values are based on the actual YARP ControlBoardVocabs.h
#define VOCAB_CM_IDLE             0x696400     // 'id\0'
#define VOCAB_CM_TORQUE           0x746f7271   // 'torq'
#define VOCAB_CM_POSITION         0x706f7300   // 'pos\0'
#define VOCAB_CM_POSITION_DIRECT  0x706f7364   // 'posd' 
#define VOCAB_CM_VELOCITY         0x76656c00   // 'vel\0'
#define VOCAB_CM_CURRENT          0x63757272   // 'curr'
#define VOCAB_CM_FORCE_IDLE       0x66696400   // 'fid\0'

// Mock base interface classes for testing
namespace yarp {
    namespace dev {
        class IControlMode {
        public:
            virtual ~IControlMode() = default;
            
            virtual bool getControlMode(int j, int* mode) = 0;
            virtual bool getControlModes(int* modes) = 0;
            virtual bool getControlModes(const int n, const int* joints, int* modes) = 0;
            virtual bool setControlMode(const int j, const int mode) = 0;
            virtual bool setControlModes(const int n, const int* joints, int* modes) = 0;
            virtual bool setControlModes(int* modes) = 0;
        };
    }
}
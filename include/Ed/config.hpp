#ifndef CONFIG_H
#define CONFIG_H

#include "Ed/lib.hpp"

namespace Ed {
//CHASSIS CONFIG
  //SET CHASSIS TYPE
  //Tank Drive = 1
  //Holonomic Drive = 2
  #define CHASSIS_TYPE 1

  //SET MOTOR PORTS
  //Negative Means Reversed
  //Tank Drive Only
  #define LEFT_MOTORS 1, 2
  #define RIGHT_MOTORS -3, -4
  //Holonomic drive only
  #define LEFT_FRONT_MOTORS 0
  #define RIGHT_FRONT_MOTORS 0
  #define RIGHT_BACK_MOTORS 0
  #define LEFT_BACK_MOTORS 0

  //SET CHASSIS CONFIGURATION
  #define CHASSIS_RPM 200
  #define CHASSIS_INPUT   1
  #define CHASSIS_OUTPUT  1
  #define WHEEL_DIAMETER 4.125
  #define CHASSIS_LENGTH 10
  #define CHASSIS_WIDTH 10

//PID CONFIG
  //SET DRIVE CONSTANTS
  #define DRIVE_KP 0
  #define DRIVE_KD 0

  //SET TURN CONSTANTS
  #define TURN_KP 0
  #define TURN_KD 0

//ODOM CONFIG
  //SET IMU PORT
  #define IMU 0

  //SET ENCODER TYPE
  //Red Quadrature Encoder = 1
  //Black Rotation Sensor  = 2
  #define ENCODER_TYPE 1

  //SET ENCODER PORTS
  //Negative Means Reversed
  //Order: Left, Right, Middle
  #define ENCODER_PORTS 0, 0, 0

  //SET TRACKING CONFIGURATION
  #define LEFT_RIGHT_DIST 0
  #define MID_CENTER_DIST 0
  #define TRACKING_DIAMETER 2.75



  inline void init() {
    switch (CHASSIS_TYPE) {
      case 1:
        tank_drive::init({LEFT_MOTORS}, {RIGHT_MOTORS},
        CHASSIS_RPM, CHASSIS_INPUT/CHASSIS_OUTPUT, WHEEL_DIAMETER, CHASSIS_WIDTH);
        break;
      case 2:
        holo_drive::init({LEFT_FRONT_MOTORS}, {RIGHT_FRONT_MOTORS}, {RIGHT_BACK_MOTORS}, {LEFT_BACK_MOTORS},
        CHASSIS_RPM, CHASSIS_INPUT/CHASSIS_OUTPUT, WHEEL_DIAMETER, CHASSIS_LENGTH, CHASSIS_WIDTH);
        break;
    }

    pid::init(DRIVE_KP, DRIVE_KD, TURN_KP, TURN_KD, CHASSIS_TYPE);

    odom::init(ENCODER_TYPE, {ENCODER_PORTS},
    IMU, LEFT_RIGHT_DIST, MID_CENTER_DIST, TRACKING_DIAMETER);
  }
}

#endif

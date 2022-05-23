#ifndef TANK_DRIVE_H
#define TANK_DRIVE_H

#include "api.h"
#include "okapi/api.hpp"

namespace tank_drive {

  //MOTORS//
  extern std::shared_ptr<okapi::MotorGroup> leftMotors;
  extern std::shared_ptr<okapi::MotorGroup> rightMotors;

  //VARIABLES//
  extern double rpm;
  extern double ratio;
  extern double wheelDiameter;
  extern double width;

  //MOVEMENT FUNCTIONS//
  void setVelocity(double leftSpeed, double rightSpeed);
  void setVoltage(double leftSpeed, double rightSpeed);

  //TELEMETRY FUNCTIONS//
  double getLeftRotation();
  double getRightRotation();
  bool isMoving();

  //CONFIGURATION FUNCTIONS//
  void resetRotation();
  void setMaxCurrent(double mA);
  void setEncoderUnits(okapi::AbstractMotor::encoderUnits unit);
  void setBrakeType(okapi::AbstractMotor::brakeMode type);
  void setDeadzone(double range);

  //DRIVER-CONTROL FUNCTIONS//
  void tankControl(double leftSpeed, double rightSpeed, bool velocity = true);
  void arcadeControl(double verticalSpeed, double horizontalSpeed, bool velocity = true);

  //INITIALIZATION//
  void init(std::initializer_list<okapi::Motor> l,
            std::initializer_list<okapi::Motor> r,
            double rpm_, double ratio_, double wheelDiameter_, double width_);

}

#endif

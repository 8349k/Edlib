#ifndef HOLO_DRIVE_H
#define HOLO_DRIVE_H

#include "api.h"
#include "okapi/api.hpp"

namespace holo_drive {

  //MOTORS//
  extern std::shared_ptr<okapi::MotorGroup> leftFrontMotors;
  extern std::shared_ptr<okapi::MotorGroup> rightFrontMotors;
  extern std::shared_ptr<okapi::MotorGroup> rightBackMotors;
  extern std::shared_ptr<okapi::MotorGroup> leftBackMotors;

  //VARIABLES//
  extern double rpm;
  extern double ratio;
  extern double wheelDiameter;
  extern double length;
  extern double width;

  //MOVEMENT FUNCTIONS//
  void setVelocity(double lf, double rf, double rb, double lb);
  void setVoltage(double lf, double rf, double rb, double lb);

  //TELEMETRY FUNCTIONS//
  double getLeftFrontRotation();
  double getRightFrontRotation();
  double getRightBackRotation();
  double getLeftBackRotation();
  bool isMoving();

  //CONFIGURATION FUNCTIONS//
  void resetRotation();
  void setMaxCurrent(double mA);
  void setEncoderUnits(okapi::AbstractMotor::encoderUnits unit);
  void setBrakeType(okapi::AbstractMotor::brakeMode type);
  void setDeadzone(double range);

  //DRIVER-CONTROL FUNCTIONS//
  void arcadeControl(double xSpeed, double ySpeed, double turnSpeed, bool velocity = true);

  //INITIALIZATION//
  void init(std::initializer_list<okapi::Motor> lf,
            std::initializer_list<okapi::Motor> rf,
            std::initializer_list<okapi::Motor> rb,
            std::initializer_list<okapi::Motor> lb,
            double rpm_, double ratio_, double wheelDiameter_, double length_, double width_);

}

#endif

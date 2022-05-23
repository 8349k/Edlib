#include "Ed/lib.hpp"

namespace tank_drive {

  //MOTORS//
  std::shared_ptr<okapi::MotorGroup> leftMotors;
  std::shared_ptr<okapi::MotorGroup> rightMotors;

  //VARIABLES//
  double rpm;
  double ratio;
  double wheelDiameter;
  double width;

  double deadzone = 0;

  //MOVEMENT FUNCTIONS//
  void setVelocity(double leftSpeed, double rightSpeed) {
    leftMotors->moveVelocity(leftSpeed*rpm/100);
    rightMotors->moveVelocity(rightSpeed*rpm/100);
  }

  void setVoltage(double leftSpeed, double rightSpeed) {
    leftMotors->moveVoltage(leftSpeed*120);
    rightMotors->moveVoltage(rightSpeed*120);
  }

  //TELEMETRY FUNCTIONS//
  double getLeftRotation() {
    return leftMotors->getPosition();
  }

  double getRightRotation() {
    return rightMotors->getPosition();
  }

  bool isMoving() {
    return !(leftMotors->isStopped() && rightMotors->isStopped());
  }

  //CONFIGURATION FUNCTIONS//
  void resetRotation() {
    leftMotors->tarePosition();
    rightMotors->tarePosition();
  }

  void setMaxCurrent(double mA) {
    leftMotors->setCurrentLimit(mA);
    rightMotors->setCurrentLimit(mA);
  }

  void setEncoderUnits(okapi::AbstractMotor::encoderUnits unit) {
    leftMotors->setEncoderUnits(unit);
    rightMotors->setEncoderUnits(unit);
  }

  void setBrakeType(okapi::AbstractMotor::brakeMode type) {
    leftMotors->setBrakeMode(type);
    rightMotors->setBrakeMode(type);
  }

  void setDeadzone(double range) {
    tank_drive::deadzone = range;
  }

  //DRIVER-CONTROL FUNCTIONS//
  #define CR *100/127
  void tankControl(double leftSpeed, double rightSpeed, bool velocity) {
    double ls = leftSpeed CR;
    double rs = rightSpeed CR;

    if (abs(ls) < deadzone) {
      ls = 0;
    }
    if (abs(rs) < deadzone) {
      rs = 0;
    }

    if (velocity) {
      setVelocity(ls, rs);
    }
    else {
      setVoltage(ls, rs);
    }
  }
  
  void arcadeControl(double verticalSpeed, double horizontalSpeed, bool velocity) {
    double vs = verticalSpeed CR;
    double hs = horizontalSpeed CR;

    if (abs(vs) < deadzone) {
      vs = 0;
    }
    if (abs(hs) < deadzone) {
      hs = 0;
    }

    if (velocity) {
      setVelocity(vs + hs, vs - hs);
    }
    else {
      setVoltage(vs + hs, vs - hs);
    }
  }

  //INITIALIZATION//
  void init(std::initializer_list<okapi::Motor> l,
            std::initializer_list<okapi::Motor> r,
            double rpm_, double ratio_, double wheelDiameter_, double width_) {

              tank_drive::rpm = rpm_;
              tank_drive::ratio = ratio_;
              tank_drive::wheelDiameter = wheelDiameter_;
              tank_drive::width = width_;

              tank_drive::leftMotors = std::make_shared<okapi::MotorGroup>(l);
              tank_drive::rightMotors = std::make_shared<okapi::MotorGroup>(r);

              tank_drive::leftMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
              tank_drive::rightMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
  }
}

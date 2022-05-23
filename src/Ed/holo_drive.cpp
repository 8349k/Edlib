#include "Ed/lib.hpp"

namespace holo_drive {

  //MOTORS//
  std::shared_ptr<okapi::MotorGroup> leftFrontMotors;
  std::shared_ptr<okapi::MotorGroup> rightFrontMotors;
  std::shared_ptr<okapi::MotorGroup> rightBackMotors;
  std::shared_ptr<okapi::MotorGroup> leftBackMotors;

  //VARIABLES//
  double rpm;
  double ratio;
  double wheelDiameter;
  double length;
  double width;

  double deadzone;

  //MOVEMENT FUNCTIONS//
  void setVelocity(double lf, double rf, double rb, double lb) {
    leftFrontMotors->moveVelocity(lf*rpm/100);
    rightFrontMotors->moveVelocity(rf*rpm/100);
    rightBackMotors->moveVelocity(rb*rpm/100);
    leftBackMotors->moveVelocity(lb*rpm/100);
  }

  void setVoltage(double lf, double rf, double rb, double lb) {
    leftFrontMotors->moveVoltage(lf*120);
    rightFrontMotors->moveVoltage(rf*120);
    rightBackMotors->moveVoltage(rb*120);
    leftBackMotors->moveVoltage(lb*120);
  }

  //TELEMETRY FUNCTIONS//
  double getLeftFrontRotation() {
    return leftFrontMotors->getPosition();
  }

  double getRightFrontRotation() {
    return rightFrontMotors->getPosition();
  }

  double getRightBackRotation() {
    return rightBackMotors->getPosition();
  }

  double getLeftBackRotation() {
    return leftBackMotors->getPosition();
  }

  bool isMoving() {
    return !(leftFrontMotors->isStopped() && rightFrontMotors->isStopped()
              && rightBackMotors->isStopped() && leftBackMotors->isStopped());
  }

  //CONFIGURATION FUNCTIONS//
  void resetRotation() {
    leftFrontMotors->tarePosition();
    rightFrontMotors->tarePosition();
    rightBackMotors->tarePosition();
    leftBackMotors->tarePosition();
  }

  void setMaxCurrent(double mA) {
    leftFrontMotors->setCurrentLimit(mA);
    rightFrontMotors->setCurrentLimit(mA);
    rightBackMotors->setCurrentLimit(mA);
    leftBackMotors->setCurrentLimit(mA);
  }

  void setEncoderUnits(okapi::AbstractMotor::encoderUnits unit) {
    leftFrontMotors->setEncoderUnits(unit);
    rightFrontMotors->setEncoderUnits(unit);
    rightBackMotors->setEncoderUnits(unit);
    leftBackMotors->setEncoderUnits(unit);
  }

  void setBrakeType(okapi::AbstractMotor::brakeMode type) {
    leftFrontMotors->setBrakeMode(type);
    rightFrontMotors->setBrakeMode(type);
    rightBackMotors->setBrakeMode(type);
    leftBackMotors->setBrakeMode(type);
  }

  void setDeadzone(double range) {
    holo_drive::deadzone = range;
  }

  //DRIVER-CONTROL FUNCTIONS//
  #define CR *100/127
  void arcadeControl(double xSpeed, double ySpeed, double turnSpeed, bool velocity) {
    double xs = xSpeed CR;
    double ys = ySpeed CR;
    double ts = turnSpeed CR;

    if (abs(xs) < holo_drive::deadzone) {
      xs = 0;
    }
    if (abs(ys) < holo_drive::deadzone) {
      ys = 0;
    }
    if (abs(ts) < holo_drive::deadzone) {
      ts = 0;
    }

    if (velocity) {
      setVelocity(xs - ys - ts,
                  xs + ys + ts,
                  xs - ys + ts,
                  xs + ys - ts);
    }
    else {
      setVoltage( xs - ys - ts,
                  xs + ys + ts,
                  xs - ys + ts,
                  xs + ys - ts);
    }
  }

  //INITIALIZATION//
  void init(std::initializer_list<okapi::Motor> lf,
            std::initializer_list<okapi::Motor> rf,
            std::initializer_list<okapi::Motor> rb,
            std::initializer_list<okapi::Motor> lb,
            double rpm_, double ratio_, double wheelDiameter_, double length_, double width_) {

              holo_drive::rpm = rpm_;
              holo_drive::ratio = ratio_;
              holo_drive::wheelDiameter = wheelDiameter_;
              holo_drive::width = width_;

              holo_drive::leftFrontMotors = std::make_shared<okapi::MotorGroup>(lf);
              holo_drive::rightFrontMotors = std::make_shared<okapi::MotorGroup>(rf);
              holo_drive::rightBackMotors = std::make_shared<okapi::MotorGroup>(rb);
              holo_drive::leftBackMotors = std::make_shared<okapi::MotorGroup>(lb);

              holo_drive::leftFrontMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
              holo_drive::rightFrontMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
              holo_drive::rightBackMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
              holo_drive::leftBackMotors->setGearing((okapi::AbstractMotor::gearset)rpm_);
  }
}

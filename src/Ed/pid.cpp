#include "lib.hpp"

namespace pid {

  //VARIABLES//
  bool usingPid;
  int driveType;

  double drive_kp;
  double drive_kd;
  double turn_kp;
  double turn_kd;

  double driveTarget = 0;
  double turnTarget = 0;
  double strafeTarget = 0;
  double xTarget = 0;
  double yTarget = 0;
  double oTarget = 0;

  //TELEMETRY FUNCTIONS//
  double getEncoderAvg() {
    return (odom::getLeftEncoder() + odom::getRightEncoder())/2;
  }

  double getChassisAvg() {
    if (driveType == 1) {
      return (tank_drive::getLeftRotation() + tank_drive::getRightRotation())/2;
    }
    else if (driveType == 2) {
      return (holo_drive::getLeftFrontRotation() + holo_drive::getRightFrontRotation())/2;
    }
    else {
      return PROS_ERR;
    }
  }

  double getChassisDiff() {
    if (driveType == 1) {
      return (tank_drive::getRightRotation() - tank_drive::getLeftRotation())/2;
    }
    else if (driveType == 2) {
      return (holo_drive::getRightFrontRotation() - holo_drive::getLeftFrontRotation())/2;
    }
    else {
      return PROS_ERR;
    }
  }

  double getStrafeAvg() {
    return (-holo_drive::getLeftFrontRotation() + holo_drive::getLeftFrontRotation())/2;
  }

  //MOVEMENT FUNCTIONS//
  void move(double dist) {
    odom::mode = 1;
    if (odom::usingOdom) {
      driveTarget = getEncoderAvg() + dist;
    }
    else {
      driveTarget = getChassisAvg() + dist;
    }
  }

  void turn(double dist) {
    odom::mode = 1;
    if (odom::usingImu) {
      turnTarget = odom::getImu() + dist;
    }
    else {
      turnTarget = getChassisDiff() + dist;
    }
  }

  void strafe(double dist) {
    if (driveType == 2) {
      odom::mode = 1;
      if (odom::usingOdom) {
        strafeTarget = odom::getMidEncoder() + dist;
      }
      else {
        strafeTarget = getStrafeAvg() + dist;
      }
    }
  }

  void turnToAngle(double angle) {
    if (odom::usingImu) {
      odom::mode = 2;
      oTarget = angle;
    }
  }

  void moveToPoint(double x, double y) {
    if (odom::usingOdom) {
      odom::mode = 2;
      xTarget = x;
      yTarget = y;
    }
  }

  void turnToPoint(double x, double y) {
    if (odom::usingOdom && odom::usingImu) {
      odom::mode = 3;
      xTarget = x;
      yTarget = y;
    }
  }

  //PID TASK//
  int pidTask() {
    struct pidValues {double drive, turn, strafe;};
    pidValues error {};
    pidValues derivative {};
    pidValues prev {0, 0, 0};
    pidValues speed {};

    while(1) {

      if (pid::driveType == 1) {

        if (odom::mode == 1) {

          if (odom::usingOdom) {
            error.drive = driveTarget - getEncoderAvg();
          }
          else {
            error.drive = driveTarget - getChassisAvg();
          }

          if (odom::usingImu) {
            error.turn = turnTarget - odom::getImu();
          }
          else {
            error.turn = turnTarget - getChassisDiff();
          }
        }

        if (odom::mode == 2) {

          if (odom::usingOdom) {
            error.drive = odom::getDistanceError(xTarget, yTarget)*odom::getDirection(xTarget, yTarget);
          }

          if (odom::usingImu) {
            error.turn = oTarget - odom::getImu();
          }
        }

        if (odom::mode == 3) {
          error.drive = 0;
          error.turn = odom::getAngleError(xTarget, yTarget)*180/M_PI;
        }

        derivative = {
          error.drive - prev.drive,
          error.turn - prev.turn
        };

        prev = {
          error.drive,
          error.turn
        };

        speed = {
          drive_kp*error.drive + drive_kd*derivative.drive,
          turn_kp*error.turn + turn_kd*derivative.turn
        };

        double leftVel = speed.drive - speed.turn;
        double rightVel = speed.drive + speed.turn;

        if (usingPid) tank_drive::setVelocity(leftVel, rightVel);
      }

      else if (pid::driveType == 2) {

        if (odom::mode == 1) {

          if (odom::usingOdom) {
            error.drive = driveTarget - getEncoderAvg();
            error.strafe = strafeTarget - odom::getMidEncoder();
          }
          else {
            error.drive = driveTarget - getChassisAvg();
            error.strafe = strafeTarget - getStrafeAvg();
          }

          if (odom::usingImu) {
            error.turn = turnTarget - odom::getImu();
          }
          else {
            error.turn = turnTarget - getChassisDiff();
          }
        }

        else if (odom::mode == 2) {

          if (odom::usingOdom) {
            double xError = xTarget - odom::getX();
            double yError = yTarget - odom::getY();
            double theta = 2*M_PI - odom::getHeading(RADIANS);

            error.drive = xError*cos(theta) - yError*sin(theta);
            error.strafe = xError*sin(theta) + yError*cos(theta);
          }

          if (odom::usingImu) {
            error.turn = oTarget - odom::getImu();
          }
        }

        if (odom::mode == 3) {
          error.drive = 0;
          error.strafe = 0;
          error.turn = odom::getAngleError(xTarget, yTarget)*180/M_PI;
        }

        derivative = {
          error.drive - prev.drive,
          error.turn - prev.turn,
          error.strafe - prev.strafe
        };

        prev = {
          error.drive,
          error.turn,
          error.strafe
        };

        speed = {
          drive_kp*error.drive + drive_kd*derivative.drive,
          turn_kp*error.turn + turn_kd*derivative.turn,
          drive_kp*error.strafe + drive_kd*derivative.strafe
        };

        double leftFrontVel = speed.drive - speed.turn - speed.strafe;
        double rightFrontVel = speed.drive + speed.turn + speed.strafe;
        double rightBackVel = speed.drive + speed.turn - speed.strafe;
        double leftBackVel = speed.drive - speed.turn + speed.strafe;

        if (usingPid) holo_drive::setVelocity(leftFrontVel, rightFrontVel, rightBackVel, leftBackVel);
      }
      pros::delay(10);
    }
    return 1;
  }

  //INITIALIZATION//
  void init(double dkp, double dkd, double tkp, double tkd, int type_) {
    pid::drive_kp = dkp;
    pid::drive_kd = dkd;
    pid::turn_kp = tkp;
    pid::turn_kd = tkd;

    if (pid::drive_kp == 0 || pid::turn_kp == 0) {
      pid::usingPid = false;
    }
    else {
      pid::usingPid = true;
      pros::Task drivePid(pidTask);
    }

    pid::driveType = type_;
  }
}

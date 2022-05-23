#include "Ed/lib.hpp"

namespace odom {

  //SENSORS//
  std::shared_ptr<okapi::ContinuousRotarySensor> leftEncoder;
  std::shared_ptr<okapi::ContinuousRotarySensor> rightEncoder;
  std::shared_ptr<okapi::ContinuousRotarySensor> midEncoder;
  std::shared_ptr<pros::Imu> imu;

  //VARIABLES//
  int mode;
  bool usingOdom;
  bool usingImu;

  double leftRightDist;
  double midCenterDist;
  double trackingDiameter;

  int direction;
  double heading;
  double prevLeftPos = 0;
  double prevRightPos = 0;
  double prevMidPos = 0;
  double prevHeading = 0;

  struct position {double x, y;};
  position global {};
  position local {};

  //FUNCTIONS//
  double getImu() {
    return -imu->get_rotation();
  }

  double getLeftEncoder() {
    return leftEncoder->get();
  }

  double getRightEncoder() {
    return rightEncoder->get();
  }

  double getMidEncoder() {
    return midEncoder->get();
  }

  double getX() {
    return global.x;
  }

  double getY() {
    return global.y;
  }

  double getHeading(bool radians) {
    if (radians)
      return heading;
    else
      return heading*180/M_PI;
  }

  double getDistanceError(double x, double y) {
    x = x - global.x;
    y = y - global.y;
    return sqrt(x*x + y*y);
  }

  double getAngleError(double x, double y) {
    x = x - global.x;
    y = y - global.y;
    if (x == 0) {
      return 0;
    }
    else {
      return atan2(y, x) - heading;
    }
  }

  int getDirection(double x, double y) {
    x = x - global.x;
    y = y - global.y;
    double theta = 2*M_PI - heading;
    double new_x = x*cos(theta) - y*sin(theta);
    if (new_x >= 0)
      direction = 1;
    else
      direction = -1;
    return direction;
  }

  //ODOM TASK//
  int odomTask() {
    global = {0, 0};

    while(1) {
      //Get position of encoders
      double leftPos = leftEncoder->get();
      double rightPos = rightEncoder->get();
      double midPos = midEncoder->get();

      //Calculate change in encoders
      double deltaLeft = leftPos - prevLeftPos;
      double deltaRight = rightPos - prevRightPos;
      double deltaMid = midPos - prevMidPos;

      //Calculate new heading
      double angle;
      double deltaAngle;
      if (usingImu) {
        angle = getImu()*M_PI/180;
        deltaAngle = heading - prevHeading;
      }
      else {
        deltaAngle = (deltaRight - deltaLeft)/(leftRightDist*2);
        angle += deltaAngle;
      }
      while(abs(angle) > 2*M_PI) {
        angle -= 2*M_PI * abs(angle)/angle;
      }
      heading = angle;

      //Store previous values
      prevLeftPos = leftPos;
      prevRightPos = rightPos;
      prevMidPos = midPos;
      prevHeading = heading;

      //Calculate local displacement
      if (deltaAngle) {
        double i = sin(deltaAngle/2)*2;
        local.x = (deltaRight/deltaAngle + leftRightDist)*i;
        local.y = (deltaMid/deltaAngle + midCenterDist)*i;
      }
      else {
        local.x = deltaRight;
        local.y = deltaMid;
      }

      double p = heading - deltaAngle/2;

      //Convert to absolute displacement
      global.x += cos(p)*local.x + sin(p)*local.y;
      global.y += cos(p)*local.y + sin(p)*local.x;

      pros::delay(10);
    }
    return 1;
  }


  //INITIALIZATION//
  std::shared_ptr<okapi::ContinuousRotarySensor>  encoder(int type, int port) {
    if (type == 1) {
      return std::make_shared<okapi::RotationSensor>(abs(port), port < 0);
    }
    else if (type == 2) {
      int p1 = abs(port); int p2 = p1++;
      return std::make_shared<okapi::ADIEncoder>(p1, p2, port < 0);
    }
    else {
      return 0;
    }
  }

  void init(int encoderType, std::array<int, 3> encoderPorts,
            int imuPort, double lrDist, double mcDist, double td) {

              odom::leftRightDist = lrDist;
              odom::midCenterDist = mcDist;
              odom::trackingDiameter = td;

              if (encoderPorts[0] != 0 && encoderPorts[1] != 0 && encoderPorts[2] != 0) {
                odom::mode = true;
                odom::usingOdom = true;
                odom::leftEncoder = encoder(encoderType, encoderPorts[0]);
                odom::rightEncoder = encoder(encoderType, encoderPorts[1]);
                odom::midEncoder = encoder(encoderType, encoderPorts[2]);
                pros::Task odometry(odom::odomTask);
              }
              else {
                odom::mode = false;
                odom::usingOdom = false;
              }

              if (imuPort != 0) {
                odom::usingImu = true;
                odom::imu = std::make_shared<pros::Imu>(imuPort);
              }
              else {
                odom::usingImu = false;
              }
  }
}

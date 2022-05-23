#ifndef ODOM_H
#define ODOM_H

#include "api.h"

namespace odom {

  //SENSORS//
  extern std::shared_ptr<okapi::ContinuousRotarySensor> leftEncoder;
  extern std::shared_ptr<okapi::ContinuousRotarySensor> rightEncoder;
  extern std::shared_ptr<okapi::ContinuousRotarySensor> midEncoder;
  extern std::shared_ptr<pros::Imu> imu;

  //VARIABLES//
  extern int mode;
  extern bool usingOdom;
  extern bool usingImu;

  //FUNCTIONS//
  double getImu();
  double getLeftEncoder();
  double getRightEncoder();
  double getMidEncoder();
  double getX();
  double getY();
  double getHeading(bool radians = false);
  double getDistanceError(double x, double y);
  double getAngleError(double x, double y);
  int getDirection(double x, double y);

  //ODOM TASK//
  int odomTask();

  //INITIALIZATION//
  std::shared_ptr<okapi::ContinuousRotarySensor>  encoder(int type, int port);
  void init(int encoderType, std::array<int, 3> encoderPorts,
            int imuPort, double lrDist, double mcDist, double td);

} //namespace odom

#endif

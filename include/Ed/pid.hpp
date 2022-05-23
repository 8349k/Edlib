#ifndef PID_H
#define PID_H

#include "api.h"

namespace pid {

  //VARIABLES//
  extern bool usingPid;
  extern int driveType;

  //MOVEMENT FUNCTIONS//
  void move(double dist);
  void turn(double dist);
  void strafe(double dist);
  void turnToAngle(double angle);
  void moveToPoint(double x, double y);
  void turnToPoint(double x, double y);

  //PID TASK//
  int pidTask();

  //INITIALIZATION//
  void init(double dkp, double dkd, double tkp, double tkd, int type_);

}

#endif

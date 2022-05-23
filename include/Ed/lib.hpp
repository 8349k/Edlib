#ifndef LIB_H
#define LIB_H

#include "api.h"

//FILES//
#include "tank_drive.hpp"
#include "holo_drive.hpp"
#include "pid.hpp"
#include "odom.hpp"

//MACROS//
#define VELOCITY true
#define VOLTAGE false

#define RADIANS true
#define DEGREES false

#define ENCODER_DEGREES okapi::AbstractMotor::encoderUnits::degrees
#define ENCODER_ROTATIONS okapi::AbstractMotor::encoderUnits::rotations
#define ENCODER_TICKS okapi::AbstractMotor::encoderUnits::counts

#define COAST okapi::AbstractMotor::brakeMode::coast
#define BRAKE okapi::AbstractMotor::brakeMode::brake
#define HOLD okapi::AbstractMotor::brakeMode::hold

#define LEFT_X master.get_analog(ANALOG_LEFT_X)
#define RIGHT_X master.get_analog(ANALOG_RIGHT_X)
#define LEFT_Y master.get_analog(ANALOG_LEFT_Y)
#define RIGHT_Y master.get_analog(ANALOG_RIGHT_Y)

#define A master.get_digital(E_CONTROLLER_DIGITAL_A);
#define B master.get_digital(E_CONTROLLER_DIGITAL_A);
#define X master.get_digital(E_CONTROLLER_DIGITAL_A);
#define Y master.get_digital(E_CONTROLLER_DIGITAL_A);
#define UP master.get_digital(E_CONTROLLER_DIGITAL_A);
#define DOWN master.get_digital(E_CONTROLLER_DIGITAL_A);
#define LEFT master.get_digital(E_CONTROLLER_DIGITAL_A);
#define RIGHT master.get_digital(E_CONTROLLER_DIGITAL_A);
#define L1 master.get_digital(E_CONTROLLER_DIGITAL_A);
#define L2 master.get_digital(E_CONTROLLER_DIGITAL_A);
#define R1 master.get_digital(E_CONTROLLER_DIGITAL_A);
#define R2 master.get_digital(E_CONTROLLER_DIGITAL_A);

#define NEW_A master.get_digital_new_press(E_CONTROLLER_DIGITAL_A);
#define NEW_B master.get_digital_new_press(E_CONTROLLER_DIGITAL_B);
#define NEW_X master.get_digital_new_press(E_CONTROLLER_DIGITAL_X);
#define NEW_Y master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y);
#define NEW_UP master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP);
#define NEW_DOWN master.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN);
#define NEW_LEFT master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT);
#define NEW_RIGHT master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT);
#define NEW_L1 master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1);
#define NEW_L2 master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2);
#define NEW_R1 master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1);
#define NEW_R2 master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2);

#endif

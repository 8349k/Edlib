#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

void initialize() {
	//Initialize Ed :)
	Ed::init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	//Disable PID task
	pid::usingPid = false;

	while (true) {
		//Set up tank controls
		tank_drive::tankControl(LEFT_Y, RIGHT_Y, VELOCITY);

		pros::delay(10);
	}
}

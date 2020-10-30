#include "main.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	strafeOdom(-24, 8, 180);
	flywheel(1000, 1);
	intakes(500, 1, 0);
	strafeOdom(-64, 8, -135);
	strafeOdom(-65, 7, -135);
	flywheel(1000, 1);
	intakes(0, 1, 0);
	strafeOdom(-48, 48, 22.5);
	strafeOdom(-32, 48, 45);
	intakes(0, 1, 0);
	strafeOdom(-24, 52, 45);
	turnOdom(180);
	intakes(500, 1, 1);
	flywheel(2500, 1);
	strafeOdom(16, 48, 90);
	intakes(3000, 1, 1);
	intakes(0, 1, 0);
	strafeOdom(95, 49, 45);
	intakes(3500, 1, 1);
	intakes(0, 1, 0);
	strafeOdom(-20, 48, -135);
	strafeOdom(-24, 82, 0);
	intakes(1500, 1, 1);
	flywheel(1500, 1);
	intakes(0, 1, 0);
	strafeOdom(-68, 89, -45);
	intakes(2000, 1, 1);
	flywheel(3000, 1);
}




/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
driveControl();
}

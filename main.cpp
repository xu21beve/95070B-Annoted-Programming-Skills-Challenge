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
	strafeOdom(-24, 8, 180);							//MOVES TO MIDDLE BOTTOM GOAL
	flywheel(1000, 1);								//PLACES PRELOAD ON TOP
	intakes(500, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(-64, 8, -135);							//MOVES TO LEFT BOTTOM GOAL TO INTAKE BALL
	strafeOdom(-65, 7, -135);							//MOVES CLOSER TO GOAL
	flywheel(1000, 1);								//OUTTAKES BALL ON TO TOP
	intakes(0, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(-48, 48, 22.5);							//MOVES TO INTAKE BALL BETWEEN LEFT MIDDLE AND CENTER GOAL
	strafeOdom(-32, 48, 45);							//MOVES TO CENTER GOAL
	intakes(0, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(-24, 52, 45);							//INTAKES THE TWO BLUE BALLS NEAR CENTER
	turnOdom(180);									//TURNS TOWARDS CENTER GOAL
	intakes(500, 1, 1);								//PLACES 3 BLUE BALLS IN THE CENTER GOAL
	flywheel(2500, 1);								//PLACES LAST BALL ON TOP
	strafeOdom(16, 48, 90);								//MOVES TO RIGHT MIDDLE GOAL
	intakes(3000, 1, 1);								//CIRCULATES BLUE BALL TO TOP
	intakes(0, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(95, 49, 45);								//MOVES TO RIGHT TOP GOAL
	intakes(3500, 1, 1);								//INTAKES RED BALL AND CIRCULATES BLUE BALL TO TOP
	intakes(0, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(-20, 48, -135);							//MOVES TO CENTER GOAL AND INTAKES RED BALL
	strafeOdom(-24, 82, 0);								//MOVES TO MIDDLE TOP GOAL
	intakes(1500, 1, 1);								//INTAKES BALLS IN GOAL WITH BLUE BALL LAST
	flywheel(1500, 1);								//PLACES BALLS IN GOAL WITH BLUE BALL ON TOP
	intakes(0, 1, 0);								//STARTS INTAKING WHILE MOVING
	strafeOdom(-68, 89, -45);							//MOVES TO LEFT TOP GOAL AND INTAKES RED BALL
	intakes(2000, 1, 1);								//INTAKES BALLS IN GOAL WITH BLUE BALL LAST
	flywheel(3000, 1);								//PLACES BALLS IN GOAL WITH BLUE BALL ON TOP
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

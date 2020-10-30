#include "main.h"

extern Controller control;
extern Motor rightFront;
extern Motor leftFront;
extern Motor leftBack;
extern Motor rightBack;
extern Motor rightIntake;
extern Motor leftIntake;
extern Motor leftFlywheel;
extern Motor rightFlywheel;
// extern Motor rightIntake;

Controller control(E_CONTROLLER_MASTER);
Motor leftFront(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
Motor rightFront(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
Motor leftBack(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
Motor rightBack(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
Motor leftIntake(5, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
Motor rightIntake(6, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
Motor leftFlywheel(7, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
Motor rightFlywheel(8, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);

#define stop control.get_digital(E_CONTROLLER_DIGITAL_R2)
#define rightB1 'A'
#define rightB2 'B'
#define leftB1 'C'
#define leftB2 'D'
#define rightF1 'E'
#define rightF2 'F'
#define leftF1 'G'
#define leftF2 'H'

okapi::ADIEncoder leftFEnc = okapi::ADIEncoder(leftF1, leftF2 ,false);
okapi::ADIEncoder rightFEnc = okapi::ADIEncoder(rightF1, rightF2, true);
okapi::ADIEncoder leftBEnc = okapi::ADIEncoder(leftB1, leftB2 ,false);
okapi::ADIEncoder rightBEnc = okapi::ADIEncoder(rightB1, rightB2, true);

#include "main.h"
#include "setup.hpp"
#include "classes.h"


double leftF = 0;
double rightF = 0;
double leftB = 0;
double rightB = 0;

double centDelta = 0;
double yDelta = 0;
double xDelta = 0;
double rDelta = 0;

double y = 0;
double x = 0;
double r = 0;

double rRad = 0;
double radToDeg = 180/M_PI;

double prevLeftF = 0;
double prevRightF = 0;
double prevLeftB = 0;
double prevRightB = 0;
double travelR;

double posSlopeAve;
double negSlopeAve;

void updateOdom(){                                                /////*        Odometry Positioning System      */////
  prevLeftF = leftF;
  prevRightF = rightF;
  prevLeftB = leftB;
  prevRightB = rightB;

  leftF = (leftFEnc.get()/360) * 3.25 * M_PI;
  rightF = (rightFEnc.get()/360)  * 3.25 * M_PI;
  leftB = (leftBEnc.get()/360) * 3.25 * M_PI;
  rightB = (rightBEnc.get()/360) * 3.25 * M_PI;

  posSlopeAve = (rightF - prevRightF + leftB - prevLeftB)/2;
  negSlopeAve = (leftF - prevLeftF + rightB - prevRightB)/2;

  double circumference = 18 * sqrt(2) * M_PI;

  rDelta = (leftF - prevLeftF - rightB + prevRightB - rightF + prevRightF + leftB - prevLeftB)/(circumference * 4) * 360;

  double turnVal = (rDelta/360) * circumference;

  centDelta = hypot(negSlopeAve - turnVal, posSlopeAve - turnVal);


  rRad += rDelta * degToRad;
  r += rDelta;
  travelR = getTravelR(posSlopeAve, negSlopeAve);

  rRad = remainder(rRad, M_PI);
  r = remainder(r, 180);

  xDelta = centDelta * sin(rRad + travelR);
  yDelta = centDelta * cos(rRad + travelR);

  x += xDelta;
  y += yDelta;
}

void driveControl(){                                          /////*            Driver Control Period           */////

  while(true){
updateOdom();

    int leftY = control.get_analog(ANALOG_LEFT_Y);
    int rightY = control.get_analog(ANALOG_RIGHT_Y);
    int leftX = control.get_analog(ANALOG_LEFT_X);
    int rightX = control.get_analog(ANALOG_RIGHT_X);
    int sign;

if(leftY == 0 && leftX == 0){
  rightFront.move(0);
  leftBack.move(0);
  leftFront.move(0);
}
else{
    if(getSign(leftY) == -1 && getSign(leftX) == -1){
      sign = -1;
    }
    else if(leftY == 0 && leftX == 0){
      sign = 0;
    }
    else{
      sign = 1;
    }

    double targetR = atan2(leftX, leftY) * sign * radToDeg;
    double travelR = targetR - r;

    double posSlopeSpeed = posSlope(travelR) * 40;
    double negSlopeSpeed = negSlope(travelR) * 40;

    rightFront.move(posSlopeSpeed);
    leftBack.move(posSlopeSpeed);
    leftFront.move(negSlopeSpeed);

    if(stop){
    leftIntake.move(0);
    rightIntake.move(0);
    leftFlywheel.move(0);
    rightFlywheel.move(0);
  }
  else{
    rightIntake.move(50);
    leftIntake.move(50);
    leftFlywheel.move(100);
    rightFlywheel.move(100);
}
  delay(10);
}
}
}
                                                                    /////*            Auton Intake Controller         */////
double intakes(double mS, double out_or_in, double simul){    /////*      1 is for intake, -1 is for outtake; 0 is for simul, 1 is for not     */////
  leftFlywheel.move(200 * out_or_in);
  rightFlywheel.move(200 * out_or_in);
  leftIntake.move(100 * out_or_in);
  rightIntake.move(100 * out_or_in);
  delay(mS * simul);
}

double flywheel(double mS, double simul){
  leftFlywheel.move(200 );
  rightFlywheel.move(200);
  delay(mS * simul);
}

double stopMotors(){
  leftFlywheel.move(0);
  rightFlywheel.move(0);
  leftIntake.move(0);
  rightIntake.move(0);
  leftFront.move(0);
  leftBack.move(0);
  rightFront.move(0);
  rightBack.move(0);
  delay(100);
}
double leftFSpeedGoal = 0;
double rightSpeedGoal = 0;
double leftBSpeedGoal = 0;
bool turnOdom(double degrees){                                 /////*           Turning Using Odometry          */////
  updateOdom();
  double diff = degrees - r;
  looper leftFPID = looper(200 * getSign(diff), 0.3, 0.01, 0.1, 40, 90, 2);    //Sets up left looper
  looper rightFPID = looper(-200 * getSign(diff), 0.3, 0.01, 0.1, 40, 90, 2);    //Sets up right looper
  looper leftBPID = looper(200 * getSign(diff), 0.3, 0.01, 0.1, 40, 90, 2);    //Sets up left looper
  looper rightBPID = looper(-200 * getSign(diff), 0.3, 0.01, 0.1, 40, 90, 2);    //Sets up right looper

  while(getSign(diff) * r < getSign(diff) * degrees){              //While the current position is not at target position
  leftFPID.update(leftFront.get_actual_velocity());                                //Update error, derivative, total error using motor encoder
  leftFSpeedGoal = leftFPID.calculateOut();                                  //Set new speed
  leftFront.move(leftFSpeedGoal);

  rightFPID.update(rightFront.get_actual_velocity());                              //Update error, derivative, total error using motor encoder
  rightSpeedGoal = rightFPID.calculateOut();                                //Set new speed
  rightFront.move(rightSpeedGoal);

  leftFPID.update(leftBack.get_actual_velocity());                                //Update error, derivative, total error using motor encoder
  leftBSpeedGoal = leftBPID.calculateOut();                                  //Set new speed
  leftBack.move(leftBSpeedGoal);

  rightBPID.update(rightBack.get_actual_velocity());                              //Update error, derivative, total error using motor encoder
  rightSpeedGoal = rightBPID.calculateOut();                                //Set new speed
  rightFront.move(rightSpeedGoal);

  delay(5);
  updateOdom();
}
stopMotors();
return false;
}
double lFSG = 0;
double rFSG = 0;
double lBSG = 0;
double rBSG = 0;
bool strafeOdom(double goalX, double goalY, double goalR){       /////*         Strafeing Using Odometry       */////
  updateOdom();
  int sign;
  double dX = goalX - x;
  double dY = goalY - y;
  int dR;
  double distance = hypot(dX, dY);
  double diff = goalR - r;

  while(getSign(dY) * goalY > getSign(dY) * y || getSign(dX) * goalX > getSign(dX) * x || getSign(diff) * r < getSign(diff) * goalR){
  dX = goalX - x;
  dY = goalY - y;
  // distance = hypot(dX, dY);
      if(getSign(dY) == -1 && getSign(dX) == -1){
        sign = -1;
      }
      else if(dY == 0 && dX == 0){
        sign = 0;
      }
      else{
        sign = 1;
      }

      double targetR = atan2(goalX, goalY) * sign * radToDeg;
      double dR = targetR - r;

      double posSlopeSpeed = posSlope(dR) * 200;
      double negSlopeSpeed = negSlope(dR) * 200;

  looper rightFPID = looper((posSlopeSpeed - 100 * getSign(diff)) * cutOff(-1, 1, distance, 5), 0.3, 0.01, 0.1, 40, 90, 2);
  rightFPID.update(rightFront.get_actual_velocity());
  rFSG = rightFPID.calculateOut();

  looper rightBPID = looper((negSlopeSpeed - 100 * getSign(diff)) * cutOff(-1, 1, distance, 5), 0.3, 0.01, 0.1, 40, 90, 2);
  rightBPID.update(rightBack.get_actual_velocity());
  rBSG = rightBPID.calculateOut();

  looper leftFPID = looper((negSlopeSpeed + 100 * getSign(diff)) * cutOff(-1, 1, distance, 5), 0.3, 0.01, 0.1, 40, 90, 2);
  leftFPID.update(leftFront.get_actual_velocity());
  lFSG = leftFPID.calculateOut();

  looper leftBPID = looper((posSlopeSpeed + 100 * getSign(diff)) * cutOff(-1, 1, distance, 5), 0.3, 0.01, 0.1, 40, 90, 2);
  leftBPID.update(leftBack.get_actual_velocity());
  lBSG = leftBPID.calculateOut();

        rightFront.move(rFSG);
        rightBack.move(rBSG);
        leftBack.move(lBSG);
        leftFront.move(lFSG);

  delay(20);

  updateOdom();
}
stopMotors();
}
                                                                  /////*      Resetting Position and direction       */////
void reset(bool xVal, bool yVal, int fieldPos){                     //fieldPos as in left or right, left = -1, right = 1
  turnOdom(0);
  double direction;
  if(yVal){
    direction = 180;
  }
  else if(xVal){
    direction = 90 * fieldPos;
  }
  rightFront.move(posSlope(direction));
  rightBack.move(negSlope(direction));
  leftFront.move(negSlope(direction));
  leftBack.move(posSlope(direction));
  resetVal(xVal, yVal);

  rightFront.move(0);
  rightBack.move(0);
  leftFront.move(0);
  leftBack.move(0);

  if(yVal){
    y = 0;
    if(remainder(r, 90) > 45){
      r = r + remainder(r, 90);
    }
    else{
      r = r - remainder(r, 90);
    }
  }
  else{
    x = 9 * 12 * fieldPos;
    if(remainder(r, 90) > 45){
      r = r + remainder(r, 90);
    }
    else{
      r = r - remainder(r, 90);
    }
  }
}

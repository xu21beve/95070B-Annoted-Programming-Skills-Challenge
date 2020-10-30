#include "main.h"

int getSign(double input){
  if(input > 0){
    return 1;
  }
  else if (input < 0){
    return -1;
  }
  else{
    return 0;
  }
}

double cutOff(double min, double max, double input, double limit){              /////*       Slowdown        */////
  if(input/limit >= max){
    return max;
  }
  else if(input/limit <= min){
    return min;
  }
  else{
    return input/limit;
  }
}

double remainder(double numerator, double denominator){
  double fract = numerator/denominator;
  int lowerInt = fract;
  double remainder = numerator - (lowerInt) * denominator * getSign(fract);
  return remainder;
}

double degToRad = M_PI/180;

double posSlope(double input){                                                  /////*       X-Drive Movement        */////
  if(input <= 0 && input >= -90){
    return 1;
  }
  else if(input >= 90 && input <= 180){
    return -1;
  }
  else if(input > 0 && input < 90){
    return cos(input * 2 * degToRad);
  }
  else{
    return cos((input + 90) * 2 * degToRad);
  }
}

double negSlope(double input){
  if(input >= 0 && input <= 90){
    return 1;
  }
  else if(input <= -90 && input >= -180){
    return -1;
  }
  else if(input < 0 && input > -90){
    return cos(input * 2 * degToRad);
  }
  else{
    return cos((input - 90) * 2 * degToRad);
  }
}

double getTravelR(double posSlopeDelta, double negSlopeDelta){
  if(posSlopeDelta <= 0 && negSlopeDelta >= 0){
    return atan2(posSlopeDelta, negSlopeDelta) + M_PI/4;
  }
  else if(posSlopeDelta <= 0 && negSlopeDelta <= 0){
    return atan2(negSlopeDelta, posSlopeDelta) + 3 * M_PI/4;
  }
  else if(posSlopeDelta >= 0 && negSlopeDelta >= 0){
    return atan2(negSlopeDelta, posSlopeDelta);
  }
  else{
    return atan2(posSlopeDelta, negSlopeDelta) - M_PI/4;
  }
}


class looper{                                                                   /////*        PID looper      */////
public:
  looper(double* input):
     valuePointer(input)
    {}
    looper(double* input, double targetValue):
     valuePointer(input), tVal(targetValue)
    {}
    looper(double* input, double targetValue, double pGain, double iGain, double dGain):
     valuePointer(input), tVal(targetValue), kP(pGain), kI(iGain), kD(dGain)
    {}
    looper(double* input, double targetValue, double pGain, double iGain, double dGain, double integralLimit, double maxLimit, double minLimit):
     valuePointer(input), tVal(targetValue), kP(pGain), kI(iGain), kD(dGain), iLim(integralLimit), maxLim(maxLimit), minLim(minLimit)
    {}
    looper(double targetValue, double pGain, double iGain, double dGain, double integralLimit, double maxLimit, double minLimit):
    tVal(targetValue), kP(pGain), kI(iGain), kD(dGain), iLim(integralLimit), maxLim(maxLimit), minLim(minLimit)
    {}

  const double kP = 0.3;
  const double kI = 0.01;
  const double kD = 0.1;

  double tVal;

  const double maxLim = 127;
  const double minLim = 10;
  const double iLim = maxLim/2;

  double error = 0;
  double integral = 0;
  double derivative = 0;
  double prevError = error;
  double out = 0;

  double* valuePointer;

void update(double inputVal) {
  prevError = error;
  error = inputVal - tVal;
  derivative = error - prevError;
  integral +=error;
}


void limits(){
  if(abs(integral) > iLim) integral = getSign(integral) * iLim;
  if(abs(out) > maxLim) out = getSign(out) * maxLim;
  if(abs(out) < minLim) out = getSign(out) * minLim;
}
double calculateOut(){
  out = (error * kP + derivative * kD + integral * kI) + tVal;
  limits();
  return out;
}
};

double resetVal(bool xVal, bool yVal){                                          ////*       Reset function      */////
  int i = 0;

  while(i < 100){
  if(int(rightFEnc.get() + leftBEnc.get() + rightBEnc.get() + leftFEnc.get() == 0)){
      delay(10);
      i++;
  }
  else{}
  }
}

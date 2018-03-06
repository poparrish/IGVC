#include <Arduino.h>
//#include <PID_v1.h>
#include "wheelControl.h"



wheelControl::wheelControl(){
  Hz = 10;
  Pgain = 0.21;//final value
  Dgain = .25;//final value
  pulsesPerRot = 45;
  RPMerror = 0;
  DerRPMerror = 0;
  lastCycleSpeedCheck = 0;
  speedCheckfloat;
  lastcycleRPM = 0;
  lastDesiredRPM = 0;
  RPMincrement = 0;
  speedCheck = 0;
  forwardBackward = false;
  backToZero = false;
  cyclesPerRot = 3336;
  RPMPlanMot = 0;
  lastEncPos = 0;
  currentPlanMotPos = 0;
  lastPlanMotPos = 0;
  deltaPlanMotPos = 0;
  derPlanMotPos = 0;
  motorSpeed = 0;
  intmotorSpeed = 0;
  lastPerror = 0;
  Perror = 0;
  Ierror = 0;
  Derror = 0;
  hubPerror = 0;
  hubIerror = 0;
  hubDerror = 0;
  lasthubPerror = 0;
  PWheelGain = 2.5;
  DWheelGain = -.05;  
  cutoff = 50;
}

returnVariables wheelControl::calculate(float newDesiredRPM, float desiredAngle, float avgDeltaTime, unsigned long lastInterrupt, int encPos) {
  using namespace std;

  /*
 * The next section looks to see if the desired RPM is in the same direction or not. IF it is not, it will set the 
 *  desired RPM to 0 this pass and then next pass if will switch the forward and back variable and the desired RPM.
 *  By setting the desried RPM to 0, we also we restart the interrupt logic and go open loop on the initial speedcheck
 *  value (set later on). 
 */
 /*
    Serial.println(newDesiredRPM);
    Serial.println(desiredAngle);
    Serial.println(avgDeltaTime);
    Serial.println(lastInterrupt);
    Serial.println(encPos);
*/

    
  if(((newDesiredRPM < 0) && (curRPM > 0)) || ((newDesiredRPM > 0) && (curRPM <0))){
      newDesiredRPM = 0;
    }
    else if(newDesiredRPM < 0){
      forwardBackward = HIGH;
    }
    else if(newDesiredRPM > 0){
      forwardBackward = LOW;
    }

    
//    if (newDesiredRPM < 0)
//    {
//      if (lastDesiredRPM > 0) {
//        desiredRPM = 0;
//        
//
//      }
//      else {
//        forwardBackward = HIGH;
//        desiredRPM = -1 * newDesiredRPM; //This is to make the rpm positive for PWM calc
//      }
//    }
//    else {
//      if (lastDesiredRPM < 0) {
//        desiredRPM = 0;
//        
//      }
//      else {
//        desiredRPM = newDesiredRPM;
//        forwardBackward = LOW;
//      }
//    }

    lastDesiredRPM = newDesiredRPM;

  /* 
 *  This if statement is looking for the case that the wheel has stopped and we are not getting any more interrupts, 
 *  and averagedeltatime is not valid (old). So, in this case we will set the current RPM to zero. If not, then we will
 *  calcultate the RPM - the 60,000 is to convert from millisec to minutes.
 */
    if (( millis() - lastInterrupt) > 100) {
      curRPM = 0;
    }
    else {
      curRPM = (1 / ((float)avgDeltaTime * pulsesPerRot)) * 60000;
    }
// IF the wheel has stopped, we need to do a new start up sequence.

    if (curRPM <= cutoff) {
      backToZero = true;
    }else{
      backToZero = false;
    }

// Calculate the delta (error) in RPM and in the derivative of RPM.

    RPMerror = desiredRPM - curRPM;
    DerRPMerror = curRPM - lastcycleRPM;
    Serial.print("curRPM:");
    Serial.print(curRPM);
    Serial.print("| Error: ");
    Serial.println(RPMerror);
    Serial.print("derrpmerror: ");
    Serial.println(DerRPMerror);
/*
 *  In the case that we are starting from zero, or our desired RPM is far from our current RPM, then we will use
 *  open loop control and set the PWM output to value that is close to the estimated PWM output for that RPM.
 *  The motor has more of a lag when it tries to slow down and hence needs a longer open loop control for this
 *  case (15 vs. 40). Also if the desired RPM is zero, then just set the PWM output to zero. The last "else"
 *  is the PD controller.
 */

    Serial.print("forwardbackward: ");
    Serial.println(forwardBackward);

    hubKp = 1.5;
    hubKd = 1;
    hubKi = .2;

    hubPerror = desiredRPM - curRPM;
    Serial.print("hubPerror: ");
    Serial.println(hubPerror);
    hubDerror = hubPerror - lasthubPerror;
    Serial.print("hubDerror: ");
    Serial.println(hubDerror);
    if(curRPM > 50){
      hubIerror += hubPerror;
    }else{
      hubIerror = 0;
    }
    Serial.print("hubIerror: ");
    Serial.println(hubIerror);
    speedCheckfloat = (hubKp * hubPerror) + (hubKd * hubDerror) + (hubKi * hubIerror);
    Serial.print("Speedcheckfloat: ");
    Serial.println(speedCheckfloat);
    speedCheck = (int)speedCheckfloat + (int)lastCycleSpeedCheck;
    speedCheck = constrain(speedCheck, 0 , 254);
    Serial.print("speedcheck: ");
    Serial.println(speedCheck);
    

   
    lasthubPerror = hubPerror;
    
//    if ((avgDeltaTime > 1000)or (backToZero) or (RPMerror > 40.0) or (RPMerror < -15.0)) {
//      if (RPMerror > 0) {
//         speedCheckfloat = ((desiredRPM * 0.9) + 7) * 0.9; //these numbers came from testing on the stand
//      }
//      else {
//         speedCheckfloat = ((desiredRPM * 0.9) + 7) * 1.1; // thesenumbers came from testing on the stand
//      }
//    }else {
//      speedCheckfloat =  RPMerror * Pgain + DerRPMerror * Dgain; 
//    }
//    if (desiredRPM <= cutoff) {
//      speedCheckfloat = 0;      
//    }
//
//    
//     lastDesiredRPM = desiredRPM;
//     lastcycleRPM = curRPM;
//
//// Limit the PWM output to 0 to 255.
//   
//    if (speedCheckfloat >= 254) {
//      speedCheckfloat = 254.0;
//    }
//     if (speedCheckfloat < -0.5) {
//      speedCheckfloat = -0.5;
//      
//    }
//    
//    lastCycleSpeedCheck = speedCheckfloat;
//    
//// this will round the float version to the integer version of the variable.
//    if(desiredRPM != 0 ){
//    speedCheck = (long)(speedCheckfloat + 0.5);
//    }
//  speedCheck = (long)speedCheckfloat;

/*
 * THis is the logic for the planetary motor - setting the angle of the wheel
 */

    deltaEncPos = (float)(encPos - lastEncPos); // change in encoder position is used for RPM
//    Serial.print("encPos: ");
//    Serial.println(encPos);
//    Serial.print("deltaEncPos: ");
//    Serial.println(deltaEncPos);
    currentPlanMotPos = lastPlanMotPos + (deltaEncPos * 360.0 / cyclesPerRot);  //calculates current position of wheel
//    Serial.print("currentPlanMotPos: ");
//    Serial.println(currentPlanMotPos);
    // We need to keep the angle between +- 180 deg
    
    if (currentPlanMotPos > 180.0) {
      currentPlanMotPos = currentPlanMotPos - 360.0;
    }
    
    if (currentPlanMotPos < -180.0) {
      currentPlanMotPos = currentPlanMotPos + 360.0;
    }

    lastPlanMotPos = currentPlanMotPos;  // used to calculate delta encoder pos, which is use to calcualte RPM
    
    RPMPlanMot = (deltaEncPos / cyclesPerRot) * (float)Hz * 60.0; //RPM calculation
    
    lastEncPos = encPos;
    
    float desiredWheelPos = desiredAngle; //Assumption that the wheel position is really the desired angle 
    float currentWheelPos = currentPlanMotPos; //Same type of assumption
    float deltaWheelPos = desiredWheelPos - currentWheelPos;  // this is the proportional error for the PD controller
    
//    this might be necessary depending upon overall wheel logic
     if (deltaWheelPos > 180.0) {
      deltaWheelPos = deltaWheelPos - 360.0;
    }
    
    if (deltaWheelPos < -180.0) {
      deltaWheelPos = deltaWheelPos + 360.0;
    }

    float RPM = RPMPlanMot;
    derPlanMotPos = RPM * 6;  //  360 deg/min / 60sec/min = deg/sec  I hope. this is the derivative error for the PD controller


    //PID IMPLEMENTATION START//
    
    float Perror, Ierror, Derror;
    float Kp=5, Ki=2, Kd=.1;
    Perror = desiredWheelPos-currentWheelPos;//calculate proportional term
    Derror = Perror - lastPerror;//calculate derivative term
    Ierror += Perror;//calculate integral term
    motorSpeed = (Kp * Perror) + (Ki * Ierror) + (Kd * Derror);//total gain (pwm written to motor)
    if((Perror < 1) && (Perror > -1)){motorSpeed = 0;}//do not increment the integral portion if +-1 degree of accuracy achieved
    motorSpeed = constrain(motorSpeed, -50, 50);//constrain floor/ceiling values to fit within +-255(8bit)
    intmotorSpeed = (long)(abs(motorSpeed));//convert to an int and add 5 for minimum power required to turn.
    lastPerror = Perror;//set last proportional term for next derivative calculation.
    if (motorSpeed >= 0) {//set appropriate direction for motors to turn since we can send negatives.
      PlanMotDir = HIGH;
    }
    else {
      PlanMotDir = LOW;
    }
    

// return the desired parameters  
  struct returnVariables returnStruct;
  returnStruct.speedCheck = speedCheck; 
  returnStruct.forwardBackward = forwardBackward;
  returnStruct.currentWheelRPM = curRPM;
  returnStruct.currentWheelAngle = currentPlanMotPos;//Changed to current
  returnStruct.motorSpeed = intmotorSpeed;
  returnStruct.planMotorDirection = PlanMotDir;
  returnStruct.backToZero = backToZero; 
  return returnStruct;   
}


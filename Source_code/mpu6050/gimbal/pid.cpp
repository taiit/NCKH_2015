/*
 * pid.cpp
 *
 * Created: 11/28/2015 5:19:51 PM
 *  Author: Vo Huu Tai
 */ 


#include <Arduino.h>
#include "pid.h"

pid::pid(){
	//some setting default
	inAuto = false;	
	pid::setOutputLimits(0, 255); //default output limit
	sampleTimeInMili = 100;	//default Controller Sample Time is 0.1 seconds
	lastTime = millis();
}

void pid::linking(float* Input, float* Output, float* Setpoint, int controlDirectrion){
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	pid::setPIDDirection(controlDirectrion);
}

bool pid::compute(){
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   
   if(timeChange >= sampleTimeInMili){// Do it every sample time..
	   
      //Compute all the working error variables
	  float input = *myInput;
      float error = *mySetpoint - input;
      ITerm += (_K_i * error);
      if(ITerm > outMax) ITerm = outMax;
      else if(ITerm < outMin) ITerm = outMin;
      float dInput = (error - lastErr);
 
      // Compute PID Output
      float output = _K_p * error  + ITerm + _K_d * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  lastErr = error;
	  //Serial.println(*myOutput,4);
	  return true;
   }
   else return false;
}

void pid::setPIDTunings(float Kp, float Ki, float Kd){
   
   if (Kp < 0 || Ki < 0 || Kd < 0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   float SampleTimeInSec = ((float)sampleTimeInMili)/1000.0;  
   _K_p = Kp;
   _K_i = Ki * SampleTimeInSec;
   _K_d = Kd / SampleTimeInSec;
 
  if(controllerDirection == REVERSE){
      _K_p = (0 - _K_p);
      _K_i = (0 - _K_i);
      _K_d = (0 - _K_d);
   }
}

void pid::setPIDSampleTime(const int ms){
   if (ms > 0){
      float ratio  = (float)ms
                      / (float)sampleTimeInMili;
      _K_i *= ratio;
      _K_d /= ratio;
      sampleTimeInMili = (unsigned long)ms;
   }
}

void pid::setOutputLimits(float Min, float Max){
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto){
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

void pid::setMode(int Mode){
    bool newAuto = (Mode == PID_MODE_RUINING);
    if(newAuto == !inAuto){  
		//we just went from manual to auto
        pid::init();
    }
    inAuto = newAuto;
}

void pid::init(){
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

void pid::setPIDDirection(int Direction){
	
   if(inAuto && Direction !=controllerDirection){
	  _K_p = (0 - _K_p);
      _K_i = (0 - _K_i);
      _K_d = (0 - _K_d);
   }   
   controllerDirection = Direction;
}

float pid::getKp(){ return  dispKp; }
float pid::getKi(){ return  dispKi;}
float pid::getKd(){ return  dispKd;}
int pid::getMode(){ return  inAuto ? PID_MODE_RUINING : MANUAL;}
int pid::getDirection(){ return controllerDirection;}

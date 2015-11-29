/*
 * pid.h
 *
 * Created: 11/28/2015 5:19:50 PM
 *  Author: Vo Huu Tai
 */ 

#ifndef PID_h
#define PID_h

class pid
{
	public:
	//Constants used in some of the functions below
	#define PID_MODE_RUINING	1
	#define MANUAL	0
	#define DIRECT  0
	#define REVERSE  1
	pid();//for array of object
	//pid(float*, float*, float*, float, float, float, int); move to linking fuinction
	void linking(float* Input, float* Output, float* Setpoint, int controlDirectrion);
	void setMode(int Mode);
	bool compute();
	void setOutputLimits(float min, float max);
	void setPIDTunings(float Kp, float Ki, float Kd);	
	void setPIDDirection(int);
	void setPIDSampleTime(const int ms);
	float getKp();
	float getKi();
	float getKd();
	int getMode();
	int getDirection();
	
	private:
	void init();
	
	float dispKp;
	float dispKi;
	float dispKd;
	
	float _K_p;
	float _K_i;
	float _K_d;

	int controllerDirection;

	float *myInput;
	float *myOutput;
	float *mySetpoint;
	// what these values are.  with pointers we'll just know.	
	unsigned long lastTime;
	float ITerm, lastInput;
	float lastErr;
	unsigned long sampleTimeInMili;
	float outMin, outMax;
	bool inAuto;
};
#endif

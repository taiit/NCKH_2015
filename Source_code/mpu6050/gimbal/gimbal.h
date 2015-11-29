/*
 * gimbal.h
 *
 * Created: 11/28/2015 2:23:05 PM
 *  Author: Vo Huu Tai
 */ 


#ifndef GIMBAL_H_
#define GIMBAL_H_

#define EVERYMS(ms) static uint16_t __CONCAT(_t,__LINE__); for(uint16_t _m = millis(); _m - __CONCAT(_t,__LINE__) >= ms; __CONCAT(_t,__LINE__) = _m)

#define MILLION 1000000.0//dt is in micro seconds
// Convert gyro values to degrees/sec
#define  FS_SEL 131.0
//#define  alpha 0.96
extern float  alpha;

#define MAX_AXIS 3
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define DEFAULT_SENSOR_BETA		0.98
#define DEFAULT_LPF_BETA_X		0.178  //default lpf_beta
#define DEFAULT_LPF_BETA_Y		0.253
#define DEFAULT_LPF_BETA_Z		0.145

extern float angle[MAX_AXIS];


//PID control

#define CUTDOWN(x,y) ((x) - (y)) > 0 ? ((x) - (y)) : 0

typedef struct pid_term_t{
	float Kp;
	float Ki;
	float Kd;
};

#endif /* GIMBAL_H_ */
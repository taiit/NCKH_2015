/*
 * gimbal.h
 *
 * Created: 11/28/2015 2:23:05 PM
 *  Author: Vo Huu Tai
 */ 


#ifndef GIMBAL_H_
#define GIMBAL_H_

#define MILLION 1000000.0//dt is in micro seconds
// Convert gyro values to degrees/sec
#define  FS_SEL 131.0
//#define  alpha 0.96
#define  alpha 0.95

#define MAX_AXIS 3
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define LPF_BETA_X 0.321
#define LPF_BETA_Y 0.223
#define LPF_BETA_Z 0.345

extern float angle[MAX_AXIS];

#endif /* GIMBAL_H_ */
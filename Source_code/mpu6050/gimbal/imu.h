/*
 * imu.h
 *
 * Created: 11/28/2015 12:53:55 PM
 *  Author: Vo Huu Tai
 */ 


#ifndef IMU_H_
#define IMU_H_
#include <Arduino.h>
#include "gimbal.h"


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally,
// and are swapped in code.
typedef union accel_t_gyro_union
{
	struct
	{
		uint8_t x_accel_h;
		uint8_t x_accel_l;
		uint8_t y_accel_h;
		uint8_t y_accel_l;
		uint8_t z_accel_h;
		uint8_t z_accel_l;
		uint8_t t_h;
		uint8_t t_l;
		uint8_t x_gyro_h;
		uint8_t x_gyro_l;
		uint8_t y_gyro_h;
		uint8_t y_gyro_l;
		uint8_t z_gyro_h;
		uint8_t z_gyro_l;
	} reg;
	struct
	{
		int x_accel;
		int y_accel;
		int z_accel;
		int temperature;
		int x_gyro;
		int y_gyro;
		int z_gyro;
	} value;
};

void calibrate_sensors();

void complement_filter(unsigned long t_now);

#endif /* IMU_H_ */
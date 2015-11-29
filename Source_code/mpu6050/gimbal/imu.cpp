/*
 * imu.cpp
 *
 * Created: 11/28/2015 1:02:15 PM
 *  Author: Vo Huu Tai
 */ 
#include <Arduino.h>
#include "imu.h"
#include "mpu6050.h"
#include "mpu6050_register.h"
#include "gimbal.h"
#include <Arduino.h>

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
static unsigned long last_read_time;
static float         last_x_angle;  // These are the filtered angles
static float         last_y_angle;
static float         last_z_angle;
static float         last_gyro_x_angle;  // Store the gyro angles to compare drift
static float         last_gyro_y_angle;
static float         last_gyro_z_angle;
//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
static float    base_x_accel = 0;
static float    base_y_accel = 0;
static float    base_z_accel = 0;

static float    base_x_gyro = 0;
static float    base_y_gyro = 0;
static float    base_z_gyro = 0;
inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

static void set_last_read_angle_data(unsigned long time_micro, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
	last_read_time = time_micro;
	last_x_angle = x;
	last_y_angle = y;
	last_z_angle = z;
	last_gyro_x_angle = x_gyro;
	last_gyro_y_angle = y_gyro;
	last_gyro_z_angle = z_gyro;
}

static int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
	// Read the raw values.
	// Read 14 bytes at once,
	// containing acceleration, temperature and gyro.
	// With the default settings of the MPU-6050,
	// there is no filter enabled, and the values
	// are not very stable.  Returns the error value
	
	accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
	
	int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

	// Swap all high and low bytes.
	// After this, the registers values are swapped,
	// so the structure name like x_accel_l does no
	// longer contain the lower byte.
	uint8_t swap;
	#define SWAP(x,y) swap = x; x = y; y = swap

	SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
	SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
	SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
	SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
	SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
	SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
	SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

	return error;
}

// The sensor should be motionless on a horizontal surface
//  while calibration is happening
void calibrate_sensors() {
	//int                   num_readings = 20;
	int                   num_readings = 1;
	float                 x_accel = 0;
	float                 y_accel = 0;
	float                 z_accel = 0;
	float                 x_gyro = 0;
	float                 y_gyro = 0;
	float                 z_gyro = 0;
	accel_t_gyro_union    accel_t_gyro;
	
	Serial.println(F(""));
	Serial.println(F("Sensor Calibrating.."));

	// Discard the first set of values read from the IMU
	read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
	
	// Read and average the raw values from the IMU
	for (int i = 0; i < num_readings; i++) {
		read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
		x_accel += accel_t_gyro.value.x_accel;
		y_accel += accel_t_gyro.value.y_accel;
		z_accel += accel_t_gyro.value.z_accel;
		x_gyro += accel_t_gyro.value.x_gyro;
		y_gyro += accel_t_gyro.value.y_gyro;
		z_gyro += accel_t_gyro.value.z_gyro;
		delay(100);
	}
	x_accel /= num_readings;
	y_accel /= num_readings;
	z_accel /= num_readings;
	x_gyro /= num_readings;
	y_gyro /= num_readings;
	z_gyro /= num_readings;
	
	// Store the raw calibration values globally
	base_x_accel = x_accel;
	base_y_accel = y_accel;
	base_z_accel = z_accel;
	base_x_gyro = x_gyro;
	base_y_gyro = y_gyro;
	base_z_gyro = z_gyro;
	// hard code
	base_x_accel = 1164.40002;
	base_y_accel = -625.40002;
	base_z_accel = 17013.59960;
	base_x_gyro = -222.30000;
	base_y_gyro = -57.75000;
	base_z_gyro = -122.90000;
	
	set_last_read_angle_data(micros(), 0, 0, 0, 0, 0, 0);
/*
	Serial.print(F("base_x_accel: ")); Serial.println(base_x_accel,5);
	Serial.print(F("base_y_accel: "));	Serial.println(base_y_accel,5);
	Serial.print(F("base_z_accel: ")); Serial.println(base_z_accel,5);
	Serial.print(F("base_x_gyro: ")); Serial.println(base_x_gyro,5);
	Serial.print(F("base_y_gyro: "));	Serial.println(base_y_gyro,5);
	Serial.print(F("base_z_gyro: ")); Serial.println(base_z_gyro,5);
*/
	Serial.println(F("Finishing Calibration"));
}

static void out_draw_data(accel_t_gyro_union accel_t_gyro){
	
	float dT = 0;
	// Print the raw acceleration values
	Serial.print(F("accel x,y,z: "));
	Serial.print(accel_t_gyro.value.x_accel, DEC);
	Serial.print(F(", "));
	Serial.print(accel_t_gyro.value.y_accel, DEC);
	Serial.print(F(", "));
	Serial.print(accel_t_gyro.value.z_accel, DEC);
	Serial.println(F(""));
 

	// The temperature sensor is -40 to +85 degrees Celsius.
	// It is a signed integer.
	// According to the datasheet: 
	// 340 per degrees Celsius, -512 at 35 degrees.
	// At 0 degrees: -512 - (340 * 35) = -12412
	Serial.print(F("temperature: "));
	dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
	Serial.print(dT, 3);
	Serial.print(F(" degrees Celsius"));
	Serial.println(F(""));
	// Print the raw gyro values.
	Serial.print(F("raw gyro x,y,z : "));
	Serial.print(accel_t_gyro.value.x_gyro, DEC);
	Serial.print(F(", "));
	Serial.print(accel_t_gyro.value.y_gyro, DEC);
	Serial.print(F(", "));
	Serial.print(accel_t_gyro.value.z_gyro, DEC);
	Serial.print(F(", "));
	Serial.println(F(""));
}

void filer_ter(unsigned long t_now){
	int error;
	
	accel_t_gyro_union accel_t_gyro;
	// Read the raw values.
	error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  
	//out_draw_data(accel_t_gyro);	
	
	float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
	float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
	float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
    
	// Get raw acceleration values
	//float G_CONVERT = 16384;
	float accel_x = accel_t_gyro.value.x_accel;
	float accel_y = accel_t_gyro.value.y_accel;
	float accel_z = accel_t_gyro.value.z_accel;
  
	// Get angle values from accelerometer	
	// float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
	float unfiltered_accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RAD_TO_DEG;
	float unfiltered_accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RAD_TO_DEG;
	float unfiltered_accel_angle_z = 0;
  
	// Compute the (filtered) gyro angles
	float dt =(t_now - get_last_time())/MILLION; // dt is in micro second
	float gyro_angle_x = gyro_x*dt + get_last_x_angle();
	float gyro_angle_y = gyro_y*dt + get_last_y_angle();
	float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
	// Compute the drifting gyro angles
	float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
	float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
	float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
	// Apply the complementary filter to figure out the change in angle - choice of alpha is
	// estimated now.  Alpha depends on the sampling rate...
	
	angle[AXIS_X] = alpha*gyro_angle_x + (1.0 - alpha)*unfiltered_accel_angle_x;
	angle[AXIS_Y] = alpha*gyro_angle_y + (1.0 - alpha)*unfiltered_accel_angle_y;
	angle[AXIS_Z] = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
	// Update the saved data with the latest values
	set_last_read_angle_data(t_now, angle[AXIS_X], angle[AXIS_Y], angle[AXIS_Z], unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
	
#if 0
	// Send the data to the serial port
	Serial.print(F("DEL:"));              //Delta T
	Serial.print(dt, DEC);
	Serial.print(F("#ACC:"));              //Accelerometer angle
	Serial.print(unfiltered_accel_angle_x, 2);
	Serial.print(F(","));
	Serial.print(unfiltered_accel_angle_y, 2);
	Serial.print(F(","));
	Serial.print(unfiltered_accel_angle_z, 2);
	Serial.print(F("#GYR:"));
	Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
	Serial.print(F(","));
	Serial.print(unfiltered_gyro_angle_y, 2);
	Serial.print(F(","));
	Serial.print(unfiltered_gyro_angle_z, 2);
	
	Serial.print(F("#FIL:"));             //Filtered angle
	Serial.print(angle_x, 2);
	Serial.print(F(","));
	Serial.print(angle_y, 2);
	Serial.print(F(","));
	Serial.print(angle_z, 2);
	Serial.println(F(""));
#endif
}
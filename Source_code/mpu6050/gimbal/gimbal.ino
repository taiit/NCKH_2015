/*
 * main.cpp
 *
 * Created: 11/28/2015 12:52:06 PM
 *  Author: Vo Huu Tai
 */
#include "imu.h"
#include "mpu6050_register.h"
#include "mpu6050.h"
#include "gimbal.h"
#include <Servo.h>
#include "lpf.h"
#include "SerialCommand.h"
#include "pid.h"

#define arduinoLED 13   // Arduino LED on board
SerialCommand SCmd;   // The demo SerialCommand object
float angle[MAX_AXIS];
Servo servo[MAX_AXIS];
LPF g_lpf[MAX_AXIS];
float  alpha = DEFAULT_SENSOR_BETA; //alpha for filer

//PID here
pid_term_t pid_term[MAX_AXIS];
//Define Variables we'll be connecting to
float my_set_point[MAX_AXIS]{0,0,0}; //it like offset
float my_input[MAX_AXIS]; //parse to angle after filter
float my_output[MAX_AXIS];//parse to angle for servo after get pid
pid pidControler[MAX_AXIS];//it is main PID
//pid myPID(&input_Y, &output_Y, &set_point_axis_Y, Kp, Ki, Kd, DIRECT);

void setup(){
	int error;
	uint8_t c;

	Serial.begin(19200);
	
	for(uint8_t i = 0; i < MAX_AXIS; i++){
		servo[i].attach(i+9);//pin 9, 10, 11 
	}
	
	// Initialize the 'Wire' class for the I2C-bus.
	init_wire();
	// default at power-up:
	//    Gyro at 250 degrees second
	//    Acceleration at 2g
	//    Clock source at internal 8MHz
	//    The device is in sleep mode.
	//
	error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
	Serial.println(F(""));
	Serial.print(F("WHO_AM_I : "));
	Serial.print(c,HEX);
	Serial.print(F(", error = "));
	Serial.println(error,DEC);
	
	// According to the datasheet, the 'sleep' bit
	// should read a '1'. But I read a '0'.
	// That bit has to be cleared, since the sensor
	// is in sleep mode at power-up. Even if the
	// bit reads '0'.
	error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
	/*
	Serial.print(F("PWR_MGMT_2 : "));
	Serial.print(c,HEX);
	Serial.print(F(", error = "));
	Serial.println(error,DEC);
	*/
	// Clear the 'sleep' bit to start the sensor.
	MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);  
	
	//Initialize the angles
	calibrate_sensors(); 
	servo[AXIS_X].write(90);
	servo[AXIS_Y].write(90);
	servo[AXIS_Z].write(90);
	
	g_lpf[AXIS_X].setBeta(DEFAULT_LPF_BETA_X);
	g_lpf[AXIS_Y].setBeta(DEFAULT_LPF_BETA_Y);
	g_lpf[AXIS_Z].setBeta(DEFAULT_LPF_BETA_Z);
	//serial command
	pinMode(arduinoLED,OUTPUT);      // Configure the onboard LED for output
	digitalWrite(arduinoLED,LOW);    // default to LED off
	// Setup callbacks for SerialCommand commands	
	SCmd.addCommand("f",			LFP_2);  // setting filter angle
	SCmd.addCommand("s",			SENSOR_FILER);  // setting filter angle
	SCmd.addCommand("default",		default_setting);
	SCmd.addCommand("stop",			stop);
	SCmd.addCommand("start",		start);
	SCmd.addCommand("pidx",			PIDX);
	SCmd.addCommand("pidy",			PIDY);
	SCmd.addDefaultHandler(			wtf);  // Handler for command that isn't matched  (says "What?")
	
	//PID here
	for(int i = 0; i < MAX_AXIS; i++){
		pidControler[i].linking(
			&my_input[i],		// pointing to data input for pid
			&my_output[i],		// pid output
			&my_set_point[i],	// set point to get error, default is Zero
			DIRECT				// non invert output
		);
		//pid term
		pid_term[i].Kp = DEFAULT_PID_P_TERM; // Is default
		pid_term[i].Ki = DEFAULT_PID_I_TERM; // Is default
		pid_term[i].Kd = DEFAULT_PID_D_TERM; // Is default
		
		pidControler[i].setPIDTunings(pid_term[i].Kp, pid_term[i].Ki, pid_term[i].Kd);
				
		pidControler[i].setOutputLimits(-90.0,90.0);	// from -90 to 90 degree
		pidControler[i].setPIDSampleTime(10);			// sampling time is 10ms
		pidControler[i].setMode(PID_MODE_RUINING);		// start pid
	}
	
}

void loop(){
	EVERYMS(10){
		
		complement_filter(micros());
		
		for(uint8_t i = 0; i < MAX_AXIS; i++){
			my_input[i]  = g_lpf[i].LPF_Caculation(angle[i]);								
		}
		
	}//End EVERYMS
	
	// Like line follow robot :D
	switch(abs((int)my_input[AXIS_X])){
		case 1: // -1 < my_input < 1
			pidControler[AXIS_X].setPIDTunings(
				CUTDOWN(pid_term[AXIS_X].Kp, 0.6),
				CUTDOWN(pid_term[AXIS_X].Ki, 1),
				CUTDOWN(pid_term[AXIS_X].Kd, 1)
			);
			break;
		case 2:
			pidControler[AXIS_X].setPIDTunings(
				CUTDOWN(pid_term[AXIS_X].Kp, 0.5),
				CUTDOWN(pid_term[AXIS_X].Ki, 1),
				CUTDOWN(pid_term[AXIS_X].Kd, 1)
			);
			break;
		case 3:
			pidControler[AXIS_X].setPIDTunings(
			CUTDOWN(pid_term[AXIS_X].Kp, 0.3),
			CUTDOWN(pid_term[AXIS_X].Ki, 0.3),
			CUTDOWN(pid_term[AXIS_X].Kd, 0.3)
			);
			break;
		case 4:
			pidControler[AXIS_X].setPIDTunings(
				CUTDOWN(pid_term[AXIS_X].Kp, 0.2),
				CUTDOWN(pid_term[AXIS_X].Ki, 0.2),
				CUTDOWN(pid_term[AXIS_X].Kd, 0.2)
			);
			break;
		default:
			pidControler[AXIS_X].setPIDTunings(
				CUTDOWN(pid_term[AXIS_X].Kp, 0.0),
				CUTDOWN(pid_term[AXIS_X].Ki, 0.0),
				CUTDOWN(pid_term[AXIS_X].Kd, 0.0)
			);
			break;
	}// end switch X
	
	switch(abs((int)my_input[AXIS_Y])){
		case 1:
			pidControler[AXIS_Y].setPIDTunings(
				CUTDOWN(pid_term[AXIS_Y].Kp, 0.4),
				CUTDOWN(pid_term[AXIS_Y].Ki, 0.4),
				CUTDOWN(pid_term[AXIS_Y].Kd, 0.4)
			);
			break;
		case 2:
			pidControler[AXIS_Y].setPIDTunings(
				CUTDOWN(pid_term[AXIS_Y].Kp, 0.3),
				CUTDOWN(pid_term[AXIS_Y].Ki, 0.3),
				CUTDOWN(pid_term[AXIS_Y].Kd, 0.3)
			);
			break;
		case 3:
			pidControler[AXIS_Y].setPIDTunings(
				CUTDOWN(pid_term[AXIS_Y].Kp, 0.2),
				CUTDOWN(pid_term[AXIS_Y].Ki, 0.2),
				CUTDOWN(pid_term[AXIS_Y].Kd, 0.2)
			);
			break;
		case 4:
			pidControler[AXIS_Y].setPIDTunings(
				CUTDOWN(pid_term[AXIS_Y].Kp, 0.1),
				CUTDOWN(pid_term[AXIS_Y].Ki, 0.1),
				CUTDOWN(pid_term[AXIS_Y].Kd, 0.1)
			);
			break;
		default:
			pidControler[AXIS_Y].setPIDTunings(
					CUTDOWN(pid_term[AXIS_Y].Kp, 0.0), 
					CUTDOWN(pid_term[AXIS_Y].Ki, 0.0),
					CUTDOWN(pid_term[AXIS_Y].Kd, 0.0)
				);
			break;
	}// end switch
	
	// Call pid
	for(uint8_t i = 0; i < MAX_AXIS; i++){
		if(pidControler[i].compute()){
			int temp = (int)(my_output[i]);
			//update pwm
			switch(i){
				case 0:
					servo[AXIS_X].write(90 + temp); // X is 0
					break;
				case 1:
					servo[AXIS_Y].write(90 + temp); // Y is 1
					break;
				case 2:
					servo[AXIS_Z].write(90 /*+ temp*/); // Z is 2
					break;
			}// end switch
			
		}// end if compute
	}//end for
	// Call serial hander
	SCmd.readSerial();  //just process serial commands	
}
//END LOOP
void out_settingdata(){
	Serial.print(F("sens alpha: "));	Serial.println(alpha);
	Serial.print(F("LPF_BETA_X: "));	Serial.println(g_lpf[AXIS_X].getBeta());
	Serial.print(F("LPF_BETA_Y: "));	Serial.println(g_lpf[AXIS_Y].getBeta());
	Serial.print(F("LPF_BETA_Z: "));	Serial.println(g_lpf[AXIS_Z].getBeta());
	Serial.println(F(""));
}
void SayHello(){
	char *arg;
	arg = SCmd.next();    // Get the next argument from the SerialCommand object buffer
	if (arg != NULL){      // As long as it existed, take it
	
		Serial.print("Hello ");
		Serial.println(arg);
	}
	else{
		Serial.println(F("Hello, whoever you are"));
	}
}
void start(){
	for(uint8_t i = 0; i < MAX_AXIS; i++){
		servo[i].attach(i+9);//pin 9, 10, 11
	}
}
void stop(){
	for(uint8_t i = 0; i < MAX_AXIS; i++){
		servo[i].detach();
	}
}
void LFP_2(){
	
	float fNum;
	char *arg;
//Filter X
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
		g_lpf[AXIS_X].setBeta(fNum);
	}
	else {
		Serial.println(F("No LPF_BETA_X"));
	}
//Filter Y
	arg = SCmd.next();
	if (arg != NULL){
		fNum=atof(arg);    // Converts a char string to a float
		g_lpf[AXIS_Y].setBeta(fNum);		
	}
	else {
		Serial.println(F("No LPF_BETA_Y"));
	}
//Filter Z
	arg = SCmd.next();
	if (arg != NULL){
		fNum=atof(arg);    // Converts a char string to a float
		g_lpf[AXIS_Z].setBeta(fNum);		
	}
	else{
		Serial.println(F("No LPF_BETA_Z"));
	}	
	out_settingdata();
}

void SENSOR_FILER(){
	float fNum;
	char *arg;
	//Filter X
	arg = SCmd.next();
	if (arg != NULL){
		fNum=atof(arg);    // Converts a char string to a float
		alpha = fNum;
	}
	else{
		Serial.println(F("No alpha"));
	}
	out_settingdata();
}

void default_setting(){
	int iNum;
	char *arg;
	//Filter X
	arg = SCmd.next();
	if (arg != NULL){
		switch(iNum){
			
			case 1:
				break;
			case 2:
				break;
			case  0:
				alpha = DEFAULT_SENSOR_BETA;
				g_lpf[AXIS_X].setBeta(DEFAULT_LPF_BETA_X);
				g_lpf[AXIS_Y].setBeta(DEFAULT_LPF_BETA_Y);
				g_lpf[AXIS_Z].setBeta(DEFAULT_LPF_BETA_Z);
				Serial.println(F("Default Setting!"));
				out_settingdata();
				break;
		}//end switch
	}//end if
}
// This gets set as the default handler, and gets called when no other command matches.
void wtf()
{
	Serial.println(F(""));
	Serial.println(F("What do you want?"));
//Print some importation
	out_settingdata();
}

void oud_pidTermData(float Kp, float Ki, float Kd){
	Serial.print(F("Kp: ")); Serial.println(Kp);
	Serial.print(F("Ki: ")); Serial.println(Ki);
	Serial.print(F("Kd: ")); Serial.println(Kd);
}
void PIDX(){
	float fNum;
	char *arg;
	//Kp
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
		//		myPID.setPIDTunings(fNum, myPID.getKi(), myPID.getKd());
	}
	else {
		oud_pidTermData(pidControler[AXIS_X].getKp(), pidControler[AXIS_X].getKi(), pidControler[AXIS_X].getKd());
	}
	//Ki
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
		//		myPID.setPIDTunings(myPID.getKp(), fNum, myPID.getKd());
	}
	else {
		Serial.println(F("No ki"));
	}
	//Kd
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
		//		myPID.setPIDTunings(myPID.getKp(), myPID.getKi(), fNum);
	}
	else {
		Serial.println(F("No kd"));
	}
}

void PIDY(){
	float fNum;
	char *arg;
//Kp
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
//		myPID.setPIDTunings(fNum, myPID.getKi(), myPID.getKd());
	}
	else {
		Serial.println(F("No kp"));
	}
//Ki
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
//		myPID.setPIDTunings(myPID.getKp(), fNum, myPID.getKd());
	}
	else {
		Serial.println(F("No ki"));
	}
//Kd
	arg = SCmd.next();
	if (arg != NULL){
		
		fNum=atof(arg);    // Converts a char string to a float
//		myPID.setPIDTunings(myPID.getKp(), myPID.getKi(), fNum);
	}
	else {
		Serial.println(F("No kd"));
	}
}

/*


float gyro_angle_x = gyro_x*dt + get_last_x_angle();
float gyro_angle_y = gyro_y*dt + get_last_y_angle();
float gyro_angle_z = gyro_z*dt + get_last_z_angle();

// Compute the drifting gyro angles
float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

// Apply the complementary filter to figure out the change in angle - choice of alpha is
// estimated now.  Alpha depends on the sampling rate...
float alpha = 0.96;
float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

// Update the saved data with the latest values
set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

// Send the data to the serial port
Serial.print(F("DEL:"));              //Delta T
Serial.print(dt, DEC);
Serial.print(F("#ACC:"));              //Accelerometer angle
Serial.print(accel_angle_x, 2);
Serial.print(F(","));
Serial.print(accel_angle_y, 2);
Serial.print(F(","));
Serial.print(accel_angle_z, 2);
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

*/



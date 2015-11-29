/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Uno, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 158
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 158
#define ARDUINO_AVR_UNO
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}

//
//
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

#include "C:\Program Files\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h" 
#include "C:\Program Files\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include <MPU6050.ino>
#include <I2C.ino>

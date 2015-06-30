# NCKH_2015
Microcontroller vs microprocess
Harvard vs Von Neumann Architecture
Avr memory architecture
	Flash/ROM: FLash vs ROM
		can be store constant variable.
	RAM/Data Memory
		Rx space
		I/O Memory
		Extended I/O Memory (MCU dependent)
		internal SRAM
	EPPROM
Bootloader
Fuse bits
Lock bits
Programmer
	usbAsp
	skt500
	USB_Bootloader
	USB_HID_Bootloader
Cygwin/gcc/g++/avr-gcc
Virtual COM PORT
	
	Herculer

**Learning C with microcontroller
	+ Use simple tool
	+ Advantage
		wariness.
	+ Disadvantage
		cant debug

** JTAG debug
	AVaRice
	purchase the JTAG-MKII or, depending on compatibility, a JTAGMKI/JTAGMKI Clone.
///
Example
	avr_0_0_coding_convention
		1. Names
		2. Function names
			+ bool b_check_for_err();
			+ void v_dump_data_to_file();
			+ is,set,get,cnt,max,min...
		3. Include units in names
			+ uint32 timer_msec;
			+ uint32 
		4. Pointer
	avr_0_1_setting_environment
		1. Compiler and IDE
			+ use atmel studio 
				>> install atmel studio
			+ use avr-gcc, makefile
				>> install WinAVR. Check System path
			+ Test with simple example.
		2. Run your code
			+ proteus simulation
			+ at16/32_dev_board
			+ Test with simple example.
		3. Debuger (*)
			+ JTAG debug (*)
			+ Test with simple example.
	avr_1_led_blink
	avr_2_IO_0
	avr_2_IO_1
	avr_3_TimerCounter_0
	avr_3_TimerCounter_1
	avr_3_TimerCounter_2

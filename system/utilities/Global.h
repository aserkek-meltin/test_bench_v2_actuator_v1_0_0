/*
 * Global.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_

#include "../../test_bench/test_bench.h"
#include <Dynamixel2Arduino.h>
#include "HardwareSerial.h"


class Global {
public:
	Global();

	//Objects
	Test_Bench			tb1;
	Test_Bench			tb2;
	Dynamixel2Arduino 	dxl;

	void GL_initialize();
};

extern Global GL;

//COMMON FUNCTIONS
float wrap360(float angle);

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_ */

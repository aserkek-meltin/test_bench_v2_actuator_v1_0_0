/*
 * Global.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_

#include "../../hand/finger/joint/joint.h"
#include <Dynamixel2Arduino.h>
#include "HardwareSerial.h"
#include "../communication/sam_pro/comm_packs.h"


class Global {
public:
	Global();

	//Objects
	Joint			tb1;
	Joint			tb2;
	Dynamixel2Arduino 	dxl;

	//Semahore Handles
	SemaphoreHandle_t 				smp_communication;
	SemaphoreHandle_t 				smp_dxl_communication;


	//COMMUNICATION PACKS-------------------------------------------------
	Sam_Command_Pack_t				sam_command_pack_t;
	Sam_Settings_Pack_t				sam_settings_pack_t;
	Sam_Status_Pack_t				sam_status_pack_t;
	Sam_Test_Pack_t					sam_test_pack_t;
	Sam_PID_Settings_Pack_t			sam_pid_settings_pack_t;
	Sam_ITA_Limits_Pack_t			sam_ita_limits_pack_t;
	SAM_Channel_t					sam_channel_t;
	int 							temp_command_recieved_counter;
	bool 							new_data;

	Error_u							errors_u;
	Status_u						status_u;
	Test_Bit_u						test_bit1_u;
	Test_Bit_u						test_bit2_u;

	void GL_initialize();
};

extern Global GL;

//COMMON FUNCTIONS
float wrap360(float angle);

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_GLOBAL_H_ */

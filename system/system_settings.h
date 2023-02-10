/*
 * system_settings.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_SYSTEM_SETTINGS_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_SYSTEM_SETTINGS_H_


//About Setup
#define NUMBER_OF_FINGERS			1
#define NUMBER_OF_TB				2
#define LENGTH_OF_LINK1_M			0.045
#define LENGTH_OF_LINK2_M			0.045
#define JOINT_PULLEY_RADIUS_M		0.001

//About Dynamixel
#define DXL_SERIAL   				Serial2
const int 	DXL_DIR_PIN 			= 2;
const float DXL_PROTOCOL_VERSION 	= 2.0;
const int DXL_CNT 					= 4;
const int DXL_LIST[DXL_CNT] 		= {1, 2, 3, 4};

//DYNAMIXEL IDs
#define FINGER_1_JOINT_1_JAA_ID		1
#define FINGER_1_JOINT_1_ITA_ID		2
#define FINGER_1_JOINT_2_JAA_ID		3
#define FINGER_1_JOINT_2_ITA_ID		4


//TODO - CRITIC - Ranges is wrong and needs to be corrected
//RANGES OF JOINTS
//TEST BENCH 1 (MP Joint)
//tb1 - jaa
#define FINGER_1_JOINT_1_JAA_MIN_MCS_DEG		176
#define FINGER_1_JOINT_1_JAA_MAX_MCS_DEG		109
//tb1 - ita
#define FINGER_1_JOINT_1_ITA_MIN_RAW			5907
#define FINGER_1_JOINT_1_ITA_MAX_RAW			104
////////////////////////////////////////////////////////////////////////////
//TEST BENCH 2 (PIP Joint)
//tb2 - jaa
#define FINGER_1_JOINT_2_JAA_MIN_MCS_DEG		150
#define FINGER_1_JOINT_2_JAA_MAX_MCS_DEG		205
//tb2 - ita
#define FINGER_1_JOINT_2_ITA_MIN_RAW		    0
#define FINGER_1_JOINT_2_ITA_MAX_RAW		    4393

#define	DEFAULT_JAA_ECS_COMMAND					0
#define DEFAULT_ITA_COMMAND						0		//TODO - CRITIC - This needs to be corrected


#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_SYSTEM_SETTINGS_H_ */

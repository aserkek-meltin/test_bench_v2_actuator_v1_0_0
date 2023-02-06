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

//About Dynamixel
#define DXL_SERIAL   				Serial2
const int 	DXL_DIR_PIN 			= 2;
const float DXL_PROTOCOL_VERSION 	= 2.0;

//DYNAMIXEL IDs
#define FINGER_1_JOINT_1_JAA_ID		1
#define FINGER_1_JOINT_1_ITA_ID		2
#define FINGER_1_JOINT_2_JAA_ID		3
#define FINGER_1_JOINT_2_ITA_ID		4

//RANGES OF JOINTS
//TEST BENCH 1 (MP Joint)
//tb1 - jaa
#define FINGER_1_JOINT_1_JAA_MIN_ECS_DEG		270
#define FINGER_1_JOINT_1_JAA_MAX_ECS_DEG		20
//tb1 - ita
#define FINGER_1_JOINT_1_ITA_MIN_RAW			1000
#define FINGER_1_JOINT_1_ITA_MAX_RAW			1000
////////////////////////////////////////////////////////////////////////////
//TEST BENCH 2 (PIP Joint)
//tb2 - jaa
#define FINGER_1_JOINT_2_JAA_MIN_ECS_DEG		270
#define FINGER_1_JOINT_2_JAA_MAX_ECS_DEG		20
//tb2 - ita
#define FINGER_1_JOINT_2_ITA_MIN_RAW		    1000
#define FINGER_1_JOINT_2_ITA_MAX_RAW		    1000


#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_SYSTEM_SETTINGS_H_ */

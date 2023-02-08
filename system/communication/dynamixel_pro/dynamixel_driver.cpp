/*
 * dynamixel_driver.cpp
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#include "dynamixel_driver.h"
#include "../../utilities/Global.h"
#include "../../system_settings.h"

bool dynamixel_initialization()
{
	bool is_initialized = true;
	GL.dxl.begin(4000000);
	GL.dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_1_JAA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_1_ITA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_2_JAA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_2_ITA_ID);

	ranges_initialization();
	return is_initialized;
}

void ranges_initialization()
{
	//TODO - FUTURE - Can take ranges from EEPROM.
	//Initialize the ranges of the Joints
	Range_t ita;
	Range_t jaa_ecs;

	//Finger1 - Joint1
	ita.min = FINGER_1_JOINT_1_ITA_MIN_RAW;
	ita.max = FINGER_1_JOINT_1_ITA_MAX_RAW;
	ita.center = (ita.min + ita.max)/2;
	jaa_ecs.min = FINGER_1_JOINT_1_JAA_MIN_ECS_DEG;
	jaa_ecs.max = FINGER_1_JOINT_1_JAA_MAX_ECS_DEG;
	jaa_ecs.center = wrap360(jaa_ecs.min + jaa_ecs.max)/2;
	GL.right_hand.index_finger.joint1.update_ranges(ita, jaa_ecs);

	//Finger2 - Joint2
	ita.min = FINGER_1_JOINT_2_ITA_MIN_RAW;
	ita.max = FINGER_1_JOINT_2_ITA_MAX_RAW;
	ita.center = (ita.min + ita.max)/2;
	jaa_ecs.min = FINGER_1_JOINT_2_JAA_MIN_ECS_DEG;
	jaa_ecs.max = FINGER_1_JOINT_2_JAA_MAX_ECS_DEG;
	jaa_ecs.center = wrap360(jaa_ecs.min + jaa_ecs.max)/2;
	GL.right_hand.index_finger.joint2.update_ranges(ita, jaa_ecs);
}

void torques_off()
{

}

void torques_on()
{

}

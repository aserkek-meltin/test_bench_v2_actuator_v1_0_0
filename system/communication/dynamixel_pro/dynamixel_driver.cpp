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

	return is_initialized;
}

void torques_off()
{

}

void torques_on()
{

}

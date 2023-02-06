/*
 * actuator_driver.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "actuator_driver.h"

#include "../system_settings.h"
#include "Global.h"

bool actuator_initialization()
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



/*
 * finger.cpp
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#include "finger.h"
#include <math.h>
#include "../../system/system_settings.h"

Finger::Finger(Finger_Settings_t finger_settings_t)
	:joint1(finger_settings_t.joint1_settings_t)
	,joint2(finger_settings_t.joint2_settings_t)
	,joint3(finger_settings_t.joint3_settings_t)
	,joint4(finger_settings_t.joint4_settings_t)
{
	finger_id = finger_settings_t.finger_id;
	fingertip_force_setpoint = 0;
}

void Finger::set_fingertip_force(float _fingertip_force_setpoint)
{
	fingertip_force_setpoint = _fingertip_force_setpoint;
}

float Finger::get_fingertip_force()
{

	return fingertip_force_setpoint;
}

void Finger::fingertip_force_2_joint_torques()
{
	float theta_1;
	float theta_2;

	float JT1 = fingertip_force_setpoint / ( (2 * LENGTH_OF_LINK1_M * cos(theta_1)) + (LENGTH_OF_LINK2_M * cos(theta_1 + theta_2)) );
	float JT2 = JT1 * (LENGTH_OF_LINK1_M*cos(theta_1)) / (LENGTH_OF_LINK2_M * cos(theta_1 + theta_2));

	joint1.set_joint_torque(JT1);
	joint2.set_joint_torque(JT2);
}

void Finger::loop()
{
	joint1.loop();
	joint2.loop();
	//joint3.loop();
	//joint4.loop();
}

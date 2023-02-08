/*
 * finger.h
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_FINGER_FINGER_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_FINGER_FINGER_H_

#include "joint/joint.h"

typedef struct
{
	uint8_t	 			finger_id;
	Joint_Settings_t 	joint1_settings_t;
	Joint_Settings_t 	joint2_settings_t;
	Joint_Settings_t 	joint3_settings_t;
	Joint_Settings_t 	joint4_settings_t;
}Finger_Settings_t;

class Finger {
public:
	Finger(Finger_Settings_t finger_settings_t);

	uint8_t				finger_id;
	Joint				joint1;
	Joint				joint2;
	Joint				joint3;
	Joint				joint4;

	void				set_fingertip_force(float _fingertip_force_setpoint);
	float				get_fingertip_force();
	void 				loop();

private:
	float 				fingertip_force_setpoint;
	void				fingertip_force_2_joint_torques();
};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_FINGER_FINGER_H_ */

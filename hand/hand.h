/*
 * hand.h
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_

#include "finger/finger.h"
#include "../system/system_settings.h"
#include "../system/communication/dynamixel_pro/dynamixel_driver.h"

typedef struct
{
	uint8_t hand_id;

	Finger_Settings_t thumb;
	Finger_Settings_t index;
	Finger_Settings_t middle;
	Finger_Settings_t ring;
	Finger_Settings_t pinky;
}Hand_Settings_t;


class Hand {
public:
	Hand(Hand_Settings_t hand_settings_t);

	uint8_t		hand_id;

	Finger		thumb_finger;
	//Finger		index_finger;
	//Finger		middle_finger;
	//Finger		ring_finger;
	//Finger		pinky_finger;


	void							recieve_all_dynamixel_commands();
	std::array<float, DXL_CNT>		get_dynamixel_commands();
	void							loop();

private:
	std::array<float, DXL_CNT> 		pwm_pos_commands;
};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_ */

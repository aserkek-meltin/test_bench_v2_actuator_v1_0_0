/*
 * hand.h
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_

#include "finger/finger.h"

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

	void		loop();
};

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_HAND_HAND_H_ */

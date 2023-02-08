/*
 * hand.cpp
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#include "hand.h"

Hand::Hand(Hand_Settings_t hand_settings_t)
	:thumb_finger	(hand_settings_t.thumb)
	,index_finger	(hand_settings_t.index)
	,middle_finger	(hand_settings_t.middle)
	,ring_finger	(hand_settings_t.ring)
	,pinky_finger	(hand_settings_t.pinky)
{
	hand_id = 		hand_settings_t.hand_id;
}

void Hand::loop(){
	thumb_finger.	loop();
	index_finger.	loop();
	middle_finger.	loop();
	ring_finger.	loop();
	pinky_finger.	loop();
}

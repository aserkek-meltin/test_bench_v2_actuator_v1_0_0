/*
 * hand.cpp
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#include "hand.h"

Hand::Hand(Hand_Settings_t hand_settings_t)
	:thumb_finger	(hand_settings_t.thumb)
	//,index_finger	(hand_settings_t.index)
	//,middle_finger	(hand_settings_t.middle)
	//,ring_finger	(hand_settings_t.ring)
	//,pinky_finger	(hand_settings_t.pinky)
{
	hand_id = 		hand_settings_t.hand_id;
	//Serial.println("Hand Constructor is called.");
	//Serial.print ("Hand :");		Serial.println (hand_id);
}

void Hand::loop(){
	//Loop for calculating required actions to control a hand
	thumb_finger.	loop();
	//index_finger.	loop();
	//middle_finger.	loop();
	//ring_finger.	loop();
	//pinky_finger.	loop();
	recieve_all_dynamixel_commands();
}

void Hand::recieve_all_dynamixel_commands()
{
	//Gathers all dynamixel angle commands from the joints to an array.
	pwm_pos_commands[0] = deg_pos2pwm_pos(thumb_finger.joint1.jaa_mcs_curr_command);
	pwm_pos_commands[1] = thumb_finger.joint1.ita_curr_command;

	pwm_pos_commands[2] = deg_pos2pwm_pos(thumb_finger.joint2.jaa_mcs_curr_command);
	pwm_pos_commands[3] = thumb_finger.joint2.ita_curr_command;
}

std::array<float, DXL_CNT> Hand::get_dynamixel_commands()
{
	//Returns the gathered dynamixel angle commands as an array.
	return pwm_pos_commands;
}

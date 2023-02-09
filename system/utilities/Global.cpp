/*
 * Global.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "Global.h"
#include "../system_settings.h"

Hand_Settings_t default_hand_settings_t = {	1,	//uint8_t hand_id;

											{},	//Finger_Settings_t thumb;
//------------------------------------------------------------------------------------------------------------------
											{	//Finger_Settings_t index;
												1, //uint8_t	 			finger_id;

												{	//Joint_Settings_t 	joint1_settings_t;
														1, //uint8_t 	joint_id;
														2, //uint8_t 	ita_id;
														1, //uint8_t 	jaa_id;
												},	//Joint_Settings_t 	joint1_settings_t;

												{	//Joint_Settings_t 	joint2_settings_t;
														2, //uint8_t 	joint_id;
														4, //uint8_t 	ita_id;
														3, //uint8_t 	jaa_id;
												},	//Joint_Settings_t 	joint2_settings_t;
												{},//Joint_Settings_t 	joint3_settings_t;
												{},	//Joint_Settings_t 	joint4_settings_t;
											},	//Finger_Settings_t index;
//------------------------------------------------------------------------------------------------------------------
											{},	//Finger_Settings_t middle;
//------------------------------------------------------------------------------------------------------------------
											{},	//Finger_Settings_t ring;
//------------------------------------------------------------------------------------------------------------------
											{}	//Finger_Settings_t pinky;
};


Global::Global()
		:right_hand(
				{	0,	//uint8_t hand_id;

						{	//Finger_Settings_t index;
							0, //uint8_t	 			finger_id;

							{	//Joint_Settings_t 	joint1_settings_t;
									0, //uint8_t 	joint_id;
									2, //uint8_t 	ita_id;
									1, //uint8_t 	jaa_id;
							},	//Joint_Settings_t 	joint1_settings_t;

							{	//Joint_Settings_t 	joint2_settings_t;
									1, //uint8_t 	joint_id;
									4, //uint8_t 	ita_id;
									3, //uint8_t 	jaa_id;
							},	//Joint_Settings_t 	joint2_settings_t;
							{},//Joint_Settings_t 	joint3_settings_t;
							{},	//Joint_Settings_t 	joint4_settings_t;
						},	//Finger_Settings_t index;
//------------------------------------------------------------------------------------------------------------------
						{	//Finger_Settings_t index;

						},	//Finger_Settings_t index;
//------------------------------------------------------------------------------------------------------------------
						{	//Finger_Settings_t index;

						},	//Finger_Settings_t index;
//------------------------------------------------------------------------------------------------------------------
						{	//Finger_Settings_t index;

						},	//Finger_Settings_t index;
//------------------------------------------------------------------------------------------------------------------
						{	//Finger_Settings_t index;

						},	//Finger_Settings_t index;
				}
		)
		, dxl(DXL_SERIAL, DXL_DIR_PIN)
{
	GL.GL_initialize();
	//Serial.println("Global Construction is called");
}

void Global::GL_initialize()
{

}

void Global::raise_command_recieved_flag_for_x_sec(int sec)
{
	if((int)(GL.temp_command_recieved_counter/sec) >= 1)
	{
		GL.status_u.status_t.COMMAND_RECIEVED = false;
	}
	else if(GL.status_u.status_t.COMMAND_RECIEVED && (int)(GL.temp_command_recieved_counter/sec) < 1)
	{
		GL.temp_command_recieved_counter++;
	}
}

float wrap360(float angle)
{
	if(angle < 0)
	{
		return wrap360(angle + 360);
	}
	else if(angle >= 360)
	{
		return wrap360(angle - 360);
	}
	else //degree >= 0 && degree < 360
	{
		return angle;
	}
}

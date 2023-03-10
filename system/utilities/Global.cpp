/*
 * Global.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "Global.h"
#include "HardwareSerial.h"
#include "../system_settings.h"
#include "../../system/communication/dynamixel_pro/dynamixel_driver.h"

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
		, fm_UARTPort(1)
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


void read_Serial_port()
{
	byte incoming;

	if (Serial.available())
	{
		bool new_pid = false;
		incoming = Serial.read();
		if (incoming == 's'){
			torques_off();
		}
		else if (incoming == 'c')
		{
			torques_on();
		}
	}
}

void raise_flag_to_send_update_pack()
{

	GL.is_uptade_needed = true;
}

void toggle_flag(uint8_t bit)
{
	if(bit == 1)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_1)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_1 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_1 = true;
		}
	}

	if(bit == 2)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_2)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_2 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_2 = true;
		}
	}

	if(bit == 3)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_3)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_3 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_3 = true;
		}
	}

	if(bit == 4)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_4)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_4 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_4 = true;
		}
	}

	if(bit == 5)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_5)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_5 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_5 = true;
		}
	}

	if(bit == 6)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_6)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_6 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_6 = true;
		}
	}

	if(bit == 7)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_7)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_7 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_7 = true;
		}
	}

	if(bit == 8)
	{
		if(GL.test_bit1_u.test_bit_t.TEST_BIT_8)
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_8 = false;
		}
		else
		{
			GL.test_bit1_u.test_bit_t.TEST_BIT_8 = true;
		}
	}

}

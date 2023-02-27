/*
 * pack_functions.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "pack_functions.h"

#include "comm_core/send_pack.h"
#include "comm_packs.h"
#include "../dynamixel_pro/dynamixel_driver.h"
//#include "../../../test_bench_actuator/controllers/internal_tension_control.h"
//#include "../../../test_bench_actuator/utilities/dynamixel.h"

void apply_commands()
{

		switch(GL.sam_command_pack_t.command)
		{
			case(SAM_CMD_ACTUATORS_TORQUES_OFF):
			{
				torques_off();

				GL.status_u.status_t.ACTUATOR_TORQUE = false;
				break;
			}
			case(SAM_CMD_ACTUATORS_TORQUES_ON):
			{
				torques_on();
				GL.status_u.status_t.ACTUATOR_TORQUE = true;
				break;
			}
			case(SAM_CMD_BWBS_CALIBRATE):
			{
				//TODO - BWBS calibrate
				break;
			}
			case(SAM_CMD_SENSORS_CALIBRATE):
			{
				//TODO - Sensors calibrate
				break;
			}
			case(SAM_CMD_DXL_POSITION_OVERWRITE_OFF):
			{
				GL.dxl_position_overwrite = false;
				break;
			}
			case(SAM_CMD_DXL_POSITION_OVERWRITE_ON):
			{
				GL.dxl_position_overwrite = true;
				break;
			}
			default:
				break;
		}
}

void apply_settings()
{

	switch(GL.sam_settings_pack_t.settings_id)
	{
		case(SAM_STG_ITA_CALIBRATE):
		{
			if(GL.sam_settings_pack_t.target_joint_id == 0)
			{
				GL.right_hand.thumb_finger.joint1.ita_calibrate();
			}
			else if(GL.sam_settings_pack_t.target_joint_id == 1)
			{
				GL.right_hand.thumb_finger.joint2.ita_calibrate();
			}
			break;
		}
		case(SAM_STG_ITA_POS_uM_SP):
		{
			if(GL.sam_settings_pack_t.target_joint_id == 0)
			{
				if(GL.right_hand.thumb_finger.joint1.get_is_ita_calibrated())
				{
					float mapped_mm = map(GL.sam_settings_pack_t.data, -7500, 7500, GL.right_hand.thumb_finger.joint1.get_ita_min(), GL.right_hand.thumb_finger.joint1.get_ita_max());
					GL.right_hand.thumb_finger.joint1.ita_curr_command = mapped_mm;
				}
				else
				{
					toggle_flag(1);
					GL.right_hand.thumb_finger.joint1.ita_curr_command = GL.right_hand.thumb_finger.joint1.get_ita_angle() + (GL.right_hand.thumb_finger.joint1.get_is_ita_sign_positive() * GL.sam_settings_pack_t.data * 4096 / 15000);
				}
			}
			else if(GL.sam_settings_pack_t.target_joint_id == 1)
			{
				if(GL.right_hand.thumb_finger.joint2.get_is_ita_calibrated())
				{
					float mapped_mm = map(GL.sam_settings_pack_t.data, -7500, 7500, GL.right_hand.thumb_finger.joint2.get_ita_min(), GL.right_hand.thumb_finger.joint2.get_ita_max());
					GL.right_hand.thumb_finger.joint2.ita_curr_command = mapped_mm;
				}
				else
				{
					GL.right_hand.thumb_finger.joint2.ita_curr_command = GL.right_hand.thumb_finger.joint2.get_ita_angle() + (GL.right_hand.thumb_finger.joint2.get_is_ita_sign_positive() * GL.sam_settings_pack_t.data * 4096 / 15000);
				}
			}
			break;
		}
		/*
		case(SAM_STG_JAA_POS_uDEG_SP):
		{

			if(GL.sam_settings_pack_t.target_joint_id == 0)
			{
				if(GL.right_hand.thumb_finger.joint1.get_is_ita_calibrated())
				{
					float mapped_mm = map(GL.sam_settings_pack_t.data, -7500, 7500, GL.right_hand.thumb_finger.joint1.get_ita_min(), GL.right_hand.thumb_finger.joint1.get_ita_max());
					GL.right_hand.thumb_finger.joint1.ita_curr_command = mapped_mm;
				}
				else
				{
					GL.right_hand.thumb_finger.joint1.ita_curr_command = GL.right_hand.thumb_finger.joint1.get_ita_angle() + (GL.right_hand.thumb_finger.joint1.get_is_ita_sign_positive() * GL.sam_settings_pack_t.data * 4096 /15000);
				}
			}
			else if(GL.sam_settings_pack_t.target_joint_id == 1)
			{
				if(GL.right_hand.thumb_finger.joint2.get_is_ita_calibrated())
				{
					float mapped_mm = map(GL.sam_settings_pack_t.data, -7500, 7500, GL.right_hand.thumb_finger.joint2.get_ita_min(), GL.right_hand.thumb_finger.joint2.get_ita_max());
					GL.right_hand.thumb_finger.joint2.ita_curr_command = mapped_mm;
				}
				else
				{
					GL.right_hand.thumb_finger.joint2.ita_curr_command = GL.right_hand.thumb_finger.joint2.get_ita_angle() + (GL.right_hand.thumb_finger.joint2.get_is_ita_sign_positive() * GL.sam_settings_pack_t.data * 4096 /15000);
				}
			}
			break;
		}
		*/
		default:
			break;
	}
}

void update_pid_coef()
{
	if(GL.sam_pid_settings_pack_t.controller_id == 0 && GL.sam_pid_settings_pack_t.target_hand_id == 0 && GL.sam_pid_settings_pack_t.target_finger_id == 0 && GL.sam_pid_settings_pack_t.target_joint_id == 0)
	{
		GL.right_hand.thumb_finger.joint1.update_it1_pid_coefficients(GL.sam_pid_settings_pack_t.kp, GL.sam_pid_settings_pack_t.ki, GL.sam_pid_settings_pack_t.kd, GL.sam_pid_settings_pack_t.kf);
	}
	else if(GL.sam_pid_settings_pack_t.controller_id == 0 && GL.sam_settings_pack_t.target_hand_id == 0 && GL.sam_settings_pack_t.target_finger_id == 0 && GL.sam_settings_pack_t.target_joint_id == 1)
	{
		GL.right_hand.thumb_finger.joint2.update_it2_pid_coefficients(GL.sam_pid_settings_pack_t.kp, GL.sam_pid_settings_pack_t.ki, GL.sam_pid_settings_pack_t.kd, GL.sam_pid_settings_pack_t.kf);
	}
	else if(GL.sam_pid_settings_pack_t.controller_id == 1 && GL.sam_pid_settings_pack_t.target_hand_id == 0 && GL.sam_pid_settings_pack_t.target_finger_id == 0 && GL.sam_pid_settings_pack_t.target_joint_id == 0)
	{
		GL.right_hand.thumb_finger.joint1.update_it1_pid_coefficients(GL.sam_pid_settings_pack_t.kp, GL.sam_pid_settings_pack_t.ki, GL.sam_pid_settings_pack_t.kd, GL.sam_pid_settings_pack_t.kf);
	}
	else if(GL.sam_pid_settings_pack_t.controller_id == 1 && GL.sam_settings_pack_t.target_hand_id == 0 && GL.sam_settings_pack_t.target_finger_id == 0 && GL.sam_settings_pack_t.target_joint_id == 1)
	{
		GL.right_hand.thumb_finger.joint2.update_it2_pid_coefficients(GL.sam_pid_settings_pack_t.kp, GL.sam_pid_settings_pack_t.ki, GL.sam_pid_settings_pack_t.kd, GL.sam_pid_settings_pack_t.kf);
	}
}

void read_serial_communication()
{
	byte buffer[255];

	xSemaphoreTake(GL.smp_sam_communication, portMAX_DELAY);
	int bytes_to_read = Serial.available();
	if(bytes_to_read > 0 && bytes_to_read < 255)
	{
		Serial.readBytes(buffer, bytes_to_read);
		GL.new_data = true;
	}
	else
	{
		//BUFFER OVERFLOW
	}
	xSemaphoreGive(GL.smp_sam_communication);

	if(GL.new_data)
	{
		GL.new_data = false;
		Catch_Data_From_Bytes(GL.sam_channel_t, buffer, bytes_to_read);
	}
}

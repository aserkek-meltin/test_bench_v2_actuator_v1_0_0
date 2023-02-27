/*
 * send_pack.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "send_pack.h"
#include "../../../utilities/Global.h"

void pack_send(	UINT8 * p_data_u8, UINT16 d_data_length, UINT32	bus)
{
	//Hatlara gore ayirabilirsin.
	//TODO - FUTURE - Gonderme fonksiyonunu yazalim.
}

void sam_pro_pack_send(const 	SAM_Pack_t * 	p_sam_pack_t,
								UINT32			bus)
{
	uint8_t tmp_u8[ SAM_PACK_MAX_LENGTH + SAM_PACK_OVERAGE] = {0};
	UINT16 i = 0;

	for(i = 0; i<SAM_PACK_HEADER + p_sam_pack_t->data_lenght_u8; i++)
	{
		tmp_u8[i] = ((UINT8 *) p_sam_pack_t)[i];
		//tmp_u8[i] = ( p_sam_pack_t)[i];
	}
	tmp_u8[i++] = p_sam_pack_t->CRC_MSByte_u8;
	tmp_u8[i++] = p_sam_pack_t->CRC_LSByte_u8;

	xSemaphoreTake(GL.smp_sam_communication, portMAX_DELAY);
	Serial.write(tmp_u8, 65);									//TODO - Critic - Burayi arttirip bakalim
	xSemaphoreGive(GL.smp_sam_communication);
}
/*
	for (int i=0; i<SAM_PACK_MAX_LENGTH + SAM_PACK_OVERAGE; i++)
	{
		Serial.print(tmp_u8[i]);
		Serial.print("\t");

	}
	Serial.println("");
*/

void send_status_pack()
{
	SAM_Pack_t 			pack_to_be_sent_t 	= {0};
	Sam_Status_Pack_t 	status_pack_t		= {0};

	//GLOBAL
	status_pack_t.system_counter_s 	= GL.system_counter_s;
	GL.errors_u.error_t.RESERVED1 = GL.dxl_position_overwrite;

	UINT8 errors = 	GL.errors_u.error_t.GFS_COULDNT_INITIALIZED 	<< 7 |
			        GL.errors_u.error_t.ITS_COULDNT_INITIALIZED 	<< 6 |
			        GL.errors_u.error_t.I2C_MUX_COULDNT_INITIALIZED	<< 5 |
			        GL.errors_u.error_t.ITA_COMMUNICATION_ERROR 	<< 4 |
			        GL.errors_u.error_t.JAA_COMMUNICATION_ERROR 	<< 3 |
			        GL.errors_u.error_t.SWB_COMMUNICATION_ERROR 	<< 2 |
			        GL.errors_u.error_t.FM_COMMUNICATION_ERROR 		<< 1 |
			        GL.errors_u.error_t.RESERVED1 						 ;


	UINT8 status = 	GL.status_u.status_t.COMMAND_RECIEVED					<< 7 |
			        GL.status_u.status_t.ITA_CALIBRATION_COMPLETED			<< 6 |
			        GL.status_u.status_t.JAA_CALIBRATION_COMPLETED			<< 5 |
			        GL.status_u.status_t.IT_PID_ON							<< 4 |
			        GL.status_u.status_t.JA_PID_ON							<< 3 |
					GL.status_u.status_t.BENDING_FRICTION_COMPENSATION_ON	<< 2 |
			        GL.status_u.status_t.ACTUATOR_TORQUE					<< 1 |
			        GL.status_u.status_t.BENDING_DIRECTION_BENDING		 		 ;

	status_pack_t.errors			= errors;
	status_pack_t.status			= status;

	//CYCLE

	switch(GL.status_pack_counter)
	{
		case STS_PACK_H1_F1_J1:
		{
			status_pack_t.joint_angle_est	= GL.right_hand.thumb_finger.joint1.get_ja_estimation();
			status_pack_t.joint_jaa_angle	= GL.right_hand.thumb_finger.joint1.get_jaa_ecs_angle();
			status_pack_t.joint_ita_mm 		= GL.right_hand.thumb_finger.joint1.get_ita_pos_mm();

			status_pack_t.joint_it1_est		= GL.right_hand.thumb_finger.joint1.get_it1();
			status_pack_t.joint_it1_sp		= GL.right_hand.thumb_finger.joint1.get_it1_setpoint();

			status_pack_t.joint_it2_est		= GL.right_hand.thumb_finger.joint1.get_it2();
			status_pack_t.joint_it2_sp		= GL.right_hand.thumb_finger.joint1.get_it2_setpoint();

			status_pack_t.test_value0_f32	= GL.right_hand.thumb_finger.joint1.ita_curr_command;
			status_pack_t.test_value1_f32 	= GL.right_hand.thumb_finger.joint1.get_ita_angle();
			status_pack_t.test_value2_f32 	= GL.sam_settings_pack_t.data;
			status_pack_t.test_value3_f32 	= 10;
			status_pack_t.test_value4_f32 	= 11;

			status_pack_t.finger_id 		= GL.right_hand.thumb_finger.finger_id;
			status_pack_t.joint_id 			= GL.right_hand.thumb_finger.joint1.joint_id;
			break;
		}
		case STS_PACK_H1_F1_J2:
		{
			status_pack_t.joint_angle_est	= GL.right_hand.thumb_finger.joint2.get_ja_estimation();
			status_pack_t.joint_jaa_angle	= GL.right_hand.thumb_finger.joint2.get_jaa_ecs_angle();
			status_pack_t.joint_ita_mm 		= GL.right_hand.thumb_finger.joint2.get_ita_pos_mm();

			status_pack_t.joint_it1_est		= GL.right_hand.thumb_finger.joint2.get_it1();
			status_pack_t.joint_it1_sp		= GL.right_hand.thumb_finger.joint2.get_it1_setpoint();

			status_pack_t.joint_it2_est		= GL.right_hand.thumb_finger.joint2.get_it2();
			status_pack_t.joint_it2_sp		= GL.right_hand.thumb_finger.joint2.get_it2_setpoint();

			status_pack_t.test_value0_f32	= GL.right_hand.thumb_finger.joint1.ita_curr_command;
			status_pack_t.test_value1_f32 	= GL.right_hand.thumb_finger.joint1.get_ita_angle();
			status_pack_t.test_value2_f32 	= GL.sam_settings_pack_t.data;
			status_pack_t.test_value3_f32 	= 10;
			status_pack_t.test_value4_f32 	= 11;

			status_pack_t.finger_id 		= GL.right_hand.thumb_finger.finger_id;
			status_pack_t.joint_id 			= GL.right_hand.thumb_finger.joint2.joint_id;
			break;
		}
	  default:
	    break;
	}
	create_sam_status_pack(&pack_to_be_sent_t, &status_pack_t);
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
	GL.status_pack_counter++;
	if(GL.status_pack_counter>1)
	{
		GL.status_pack_counter = 0;
	}
}

void send_update_pack()
{
	SAM_Pack_t 			pack_to_be_sent_t 	= {0};
	Sam_Update_Pack_t 	update_pack_t		= {0};

	//GLOBAL
	UINT8 test_bits1 = 	GL.test_bit1_u.test_bit_t.TEST_BIT_1			 << 7 |
			        	GL.test_bit1_u.test_bit_t.TEST_BIT_2             << 6 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_3             << 5 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_4             << 4 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_5             << 3 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_6             << 2 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_7             << 1 |
						GL.test_bit1_u.test_bit_t.TEST_BIT_8             	  ;

	UINT8 test_bits2 = 	GL.test_bit2_u.test_bit_t.TEST_BIT_1			 << 7 |
			        	GL.test_bit2_u.test_bit_t.TEST_BIT_2             << 6 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_3             << 5 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_4             << 4 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_5             << 3 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_6             << 2 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_7             << 1 |
						GL.test_bit2_u.test_bit_t.TEST_BIT_8             	  ;


	update_pack_t.test_value1_u32 		= GL.system_counter_s; //GL.sam_test_pack_t.test_value1_u32;
	update_pack_t.test_bits1			= test_bits1;
	update_pack_t.test_bits2			= test_bits2;

	//CYCLE
	switch(GL.update_pack_counter)
	{
		case STS_PACK_H1_F1_J1:
		{

			update_pack_t.joint_jaa_min			= GL.right_hand.thumb_finger.joint1.get_jaa_min();
			update_pack_t.joint_jaa_max			= GL.right_hand.thumb_finger.joint1.get_jaa_max();
			update_pack_t.joint_ita_min			= GL.right_hand.thumb_finger.joint1.get_ita_min();
			update_pack_t.joint_ita_max			= GL.right_hand.thumb_finger.joint1.get_ita_max();
			update_pack_t.joint_torque_sp		= GL.right_hand.thumb_finger.joint1.get_joint_torque_setpoint();
			update_pack_t.test_value6_f32		= 0;
			update_pack_t.test_value7_f32		= 0;
			update_pack_t.test_value8_f32		= 0;
			update_pack_t.test_value9_f32		= 0;
			update_pack_t.test_value10_f32 		= 0;
			update_pack_t.test_value11_f32 		= 0;
			update_pack_t.test_value12_f32 		= 0;

			update_pack_t.finger_id 			= GL.right_hand.thumb_finger.finger_id;
			update_pack_t.joint_id 				= GL.right_hand.thumb_finger.joint1.joint_id;
			break;
		}
		case STS_PACK_H1_F1_J2:
		{
			update_pack_t.joint_jaa_min			= GL.right_hand.thumb_finger.joint2.get_jaa_min();
			update_pack_t.joint_jaa_max			= GL.right_hand.thumb_finger.joint2.get_jaa_max();
			update_pack_t.joint_ita_min			= GL.right_hand.thumb_finger.joint2.get_ita_min();
			update_pack_t.joint_ita_max			= GL.right_hand.thumb_finger.joint2.get_ita_max();
			update_pack_t.joint_torque_sp		= GL.right_hand.thumb_finger.joint2.get_joint_torque_setpoint();
			update_pack_t.test_value6_f32		= 0;
			update_pack_t.test_value7_f32		= 0;
			update_pack_t.test_value8_f32		= 0;
			update_pack_t.test_value9_f32		= 0;
			update_pack_t.test_value10_f32 		= 0;
			update_pack_t.test_value11_f32 		= 0;
			update_pack_t.test_value12_f32 		= 0;

			update_pack_t.finger_id 			= GL.right_hand.thumb_finger.finger_id;
			update_pack_t.joint_id 				= GL.right_hand.thumb_finger.joint1.joint_id;
			break;
		}
		default:
			break;
	}

	//TODO - Communication - Paketin icini dolduralim.

	create_sam_update_pack	(&pack_to_be_sent_t, &update_pack_t);
	Pack_It_SAM				(&pack_to_be_sent_t);
	sam_pro_pack_send		(&pack_to_be_sent_t, 1);
	GL.update_pack_counter++;
	if(GL.update_pack_counter>1)
	{
		GL.update_pack_counter = 0;
		GL.is_uptade_needed = false;
	}
}

void send_pid_coeff_pack(Sam_PID_Settings_Pack_t sam_pid_settings_pack_t)
{
	SAM_Pack_t 					pack_to_be_sent_t 	= {0};
	Sam_PID_Settings_Pack_t		pid_settings_pack_t	= {0};

	pid_settings_pack_t.controller_id		= sam_pid_settings_pack_t.controller_id;
	pid_settings_pack_t.target_hand_id		= sam_pid_settings_pack_t.target_hand_id;
	pid_settings_pack_t.target_finger_id	= sam_pid_settings_pack_t.target_finger_id;
	pid_settings_pack_t.target_joint_id		= sam_pid_settings_pack_t.target_joint_id;

	if( pid_settings_pack_t.target_hand_id == 0 && pid_settings_pack_t.target_finger_id == 0 && pid_settings_pack_t.target_joint_id == 0 && pid_settings_pack_t.controller_id == IT1_PID )
	{
		std::array<int,4> coeffs;
		coeffs = GL.right_hand.thumb_finger.joint1.get_it1_pidf_coeff();

		pid_settings_pack_t.kp			= coeffs[0];
		pid_settings_pack_t.ki			= coeffs[1];
		pid_settings_pack_t.kd			= coeffs[2];
		pid_settings_pack_t.kf			= coeffs[3];
	}
	else if(pid_settings_pack_t.target_hand_id == 0 && pid_settings_pack_t.target_finger_id == 0 && pid_settings_pack_t.target_joint_id == 0 && pid_settings_pack_t.controller_id == IT2_PID )
	{
		std::array<int,4> coeffs;
		coeffs = GL.right_hand.thumb_finger.joint1.get_it2_pidf_coeff();

		pid_settings_pack_t.kp			= coeffs[0];
		pid_settings_pack_t.ki			= coeffs[1];
		pid_settings_pack_t.kd			= coeffs[2];
		pid_settings_pack_t.kf			= coeffs[3];
	}
	else if( pid_settings_pack_t.target_hand_id == 0 && pid_settings_pack_t.target_finger_id == 0 && pid_settings_pack_t.target_joint_id == 1 && pid_settings_pack_t.controller_id == IT1_PID )
	{
		std::array<int,4> coeffs;
		coeffs = GL.right_hand.thumb_finger.joint2.get_it1_pidf_coeff();

		pid_settings_pack_t.kp			= coeffs[0];
		pid_settings_pack_t.ki			= coeffs[1];
		pid_settings_pack_t.kd			= coeffs[2];
		pid_settings_pack_t.kf			= coeffs[3];
	}
	else if(pid_settings_pack_t.target_hand_id == 0 && pid_settings_pack_t.target_finger_id == 0 && pid_settings_pack_t.target_joint_id == 1 && pid_settings_pack_t.controller_id == IT2_PID )
	{
		std::array<int,4> coeffs;
		coeffs = GL.right_hand.thumb_finger.joint2.get_it2_pidf_coeff();

		pid_settings_pack_t.kp			= coeffs[0];
		pid_settings_pack_t.ki			= coeffs[1];
		pid_settings_pack_t.kd			= coeffs[2];
		pid_settings_pack_t.kf			= coeffs[3];
	}
/*
	else if(_pid_id == IT1_PID)
	{
		pid_settings_pack_t.kp			= GL.it1_pid_settings_t.Kp;
		pid_settings_pack_t.ki			= GL.it1_pid_settings_t.Ki;
		pid_settings_pack_t.kd			= GL.it1_pid_settings_t.Kd;
	}
	else if(_pid_id == IT2_PID)
	{
		pid_settings_pack_t.kp			= GL.it2_pid_settings_t.Kp;
		pid_settings_pack_t.ki			= GL.it2_pid_settings_t.Ki;
		pid_settings_pack_t.kd			= GL.it2_pid_settings_t.Kd;
	}
	else if (_pid_id == JA_PID_WFF)
	{
		pid_settings_pack_t.kp			= GL.ja_pid_wff_settings_t.Kp;
		pid_settings_pack_t.ki			= GL.ja_pid_wff_settings_t.Ki;
		pid_settings_pack_t.kd			= GL.ja_pid_wff_settings_t.Kd;
		pid_settings_pack_t.kf			= GL.ja_pid_wff_settings_t.Kf;
	}

	create_sam_pid_settings_pack(&pack_to_be_sent_t, &pid_settings_pack_t);
	*/
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
}




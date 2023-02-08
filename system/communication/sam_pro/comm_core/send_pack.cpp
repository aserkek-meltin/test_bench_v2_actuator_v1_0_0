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
	Serial.write(tmp_u8, 65);
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
/*
	//TODO - Communication - Paketin icini dolduralim.
	status_pack_t.system_counter_s 	= GL.system_counter_s;

	status_pack_t.errors			= errors;
	status_pack_t.status			= status;

	status_pack_t.ja 				= GL.ja_data_t.as5048a_data_raw_ecs;
	status_pack_t.ja_setpoint		= GL.ja_pid_settings_t.pid_setpoint;

	status_pack_t.it 				= GL.it_data_t.converted;
	status_pack_t.it_setpoint		= GL.it_pid_settings_t.user_setpoint_SI;

	status_pack_t.bwb				= GL.bwbs_data_t.ma;
	status_pack_t.bending_estimation= GL.bending_angle_estimator_t.bending_angle_result;

	status_pack_t.ftf				= GL.gfs_data_t.converted;	//GL.fingertip_force_estimator_t.ftf_model1;
	status_pack_t.ftf_setpoint		= GL.ftf_pid_settings_t.user_setpoint_SI;

	//status_pack_t.test_value1_f32 = GL.bending_angle_estimator_t.bending_angle_model1;
	status_pack_t.test_value1_f32 = GL.jaa_ecs.curr_command;
	status_pack_t.test_value2_f32 = GL.jaa_ecs.current;
	status_pack_t.test_value3_f32 = GL.ftf_controller_t.it1_estimated_N;
	status_pack_t.test_value4_f32 = GL.ftf_controller_t.it2_estimated_N;

	status_pack_t.test_value5_u8 = GL.ftf_controller_t.it1_direction;
	status_pack_t.test_value6_u8 = GL.ftf_controller_t.it2_direction;

	GL.status_u.status_t.IT_PID_ON = GL.it_pid_settings_t.is_control_on;
	GL.status_u.status_t.JA_PID_ON = GL.ja_pid_settings_t.is_control_on;
*/

	UINT8 errors = 	GL.errors_u.error_t.GFS_COULDNT_INITIALIZED 	<< 7 |
			        GL.errors_u.error_t.ITS_COULDNT_INITIALIZED 	<< 6 |
			        GL.errors_u.error_t.BENDING_ESTIMATION_NOT_SURE	<< 5 |
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


	create_sam_status_pack(&pack_to_be_sent_t, &status_pack_t);
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
}

void send_test_pack()
{
	SAM_Pack_t 			pack_to_be_sent_t 	= {0};
	Sam_Test_Pack_t 	test_pack_t			= {0};
/*
	//TODO - Communication - Paketin icini dolduralim.
	test_pack_t.test_value1_u32 = GL.system_counter_s; //GL.sam_test_pack_t.test_value1_u32;

	test_pack_t.test_bits1			= test_bits1;
	test_pack_t.test_bits2			= test_bits2;

	test_pack_t.test_value1_f32 	= GL.ita_range_t.min;
	test_pack_t.test_value2_f32 	= GL.ita_range_t.max;
	test_pack_t.test_value3_f32 	= GL.it_data_t.zero;
	test_pack_t.test_value4_f32 	= GL.it_pid_settings_t.pid_setpoint;
	test_pack_t.test_value5_f32 	= GL.bending_angle_estimator_t.bending_angle_zero;
	test_pack_t.test_value6_f32 	= GL.ita.prev_command; //GL.sam_test_pack_t.test_value6_f32; //GL.bending_angle_estimator_t.bending_angle_model1;
	test_pack_t.test_value7_f32 	= GL.jaa_mcs.prev_command; //GL.sam_test_pack_t.test_value7_f32; //GL.bending_angle_estimator_t.bending_angle_model2;
	test_pack_t.test_value8_f32 	= GL.gfs_data_t.ma;
	test_pack_t.test_value9_f32 	= GL.ja_pid_wff_settings_t.input;				//GL.ftf_controller_t.it1_compensated_N;
	test_pack_t.test_value10_f32 	= GL.ja_pid_wff_settings_t.user_setpoint_SI;	//GL.ftf_controller_t.it2_compensated_N;
	test_pack_t.test_value11_f32 	= GL.ja_pid_wff_settings_t.pid_setpoint;		//GL.ftf_controller_t.it1_out_desired_N;
	test_pack_t.test_value12_f32 	= GL.ja_pid_wff_settings_t.output;				//GL.ftf_controller_t.it2_out_desired_N;
*/
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


	test_pack_t.test_value1_u8 		= GL.sam_test_pack_t.test_value1_u8;
	test_pack_t.test_value2_u8 		= GL.sam_test_pack_t.test_value2_u8;;

	create_sam_test_pack(&pack_to_be_sent_t, &test_pack_t);
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
}

void send_pid_coeff_pack(UINT8 _pid_id)
{
	SAM_Pack_t 					pack_to_be_sent_t 	= {0};
	Sam_PID_Settings_Pack_t		pid_settings_pack_t	= {0};

	pid_settings_pack_t.pid_id		= _pid_id;
/*
	if(_pid_id == IT_PID)
	{
		pid_settings_pack_t.kp			= GL.it_pid_settings_t.Kp;
		pid_settings_pack_t.ki			= GL.it_pid_settings_t.Ki;
		pid_settings_pack_t.kd			= GL.it_pid_settings_t.Kd;
	}
	else if(_pid_id == JA_PID)
	{
		pid_settings_pack_t.kp			= GL.ja_pid_settings_t.Kp;
		pid_settings_pack_t.ki			= GL.ja_pid_settings_t.Ki;
		pid_settings_pack_t.kd			= GL.ja_pid_settings_t.Kd;
	}
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
*/

	create_sam_pid_settings_pack(&pack_to_be_sent_t, &pid_settings_pack_t);
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
}

void send_ita_limits_pack()
{
	SAM_Pack_t 					pack_to_be_sent_t 	= {0};
	Sam_ITA_Limits_Pack_t		sam_ita_limits_pack = {0};
	Sam_PID_Settings_Pack_t		pid_settings_pack_t	= {0};
/*
	sam_ita_limits_pack.ita_min = GL.ita_range_t.min;
	sam_ita_limits_pack.ita_max = GL.ita_range_t.max;
	sam_ita_limits_pack.ita_current = GL.ita.current;
*/
	create_sam_ita_limits_pack(&pack_to_be_sent_t, &sam_ita_limits_pack);
	Pack_It_SAM(&pack_to_be_sent_t);
	sam_pro_pack_send(&pack_to_be_sent_t, 1);
}


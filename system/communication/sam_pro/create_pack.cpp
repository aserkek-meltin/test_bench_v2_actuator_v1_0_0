/*
 * create_pack.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "create_pack.h"

#include "HardwareSerial.h"

void create_sam_status_pack(		SAM_Pack_t			*p_sam_pack_t,
							const 	Sam_Status_Pack_t 	*sam_status_pack_t)
{
	UINT16 pack_index_u16 = 0;

	UINT32_send (p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->system_counter_s);

	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->errors);
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->status);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->ja);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->ja_setpoint);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->it);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->it_setpoint);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->bwb);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->bending_estimation);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->ftf);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->ftf_setpoint);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value1_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value2_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value3_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value4_f32);
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value5_u8);
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_status_pack_t->test_value6_u8);

	p_sam_pack_t->data_lenght_u8 = pack_index_u16;
	p_sam_pack_t->pack_type_u8 = STATUS_PACK;
}

void create_sam_test_pack(			SAM_Pack_t			*p_sam_pack_t,
							const 	Sam_Test_Pack_t 	*sam_test_pack_t)
{
	UINT16 pack_index_u16 = 0;

	UINT32_send (p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value1_u32);

	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_bits1);
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_bits2);

	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value1_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value2_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value3_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value4_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value5_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value6_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value7_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value8_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value9_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value10_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value11_f32);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value12_f32);

	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value1_u8);
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_test_pack_t->test_value2_u8);

	p_sam_pack_t->data_lenght_u8 = pack_index_u16;
	p_sam_pack_t->pack_type_u8 = TEST_PACK;
}

void create_sam_pid_settings_pack(		SAM_Pack_t					*p_sam_pack_t,
								const 	Sam_PID_Settings_Pack_t 	*sam_pid_settings_pack_t)
{
	UINT16 pack_index_u16 = 0;
	UINT8_send  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->pid_id);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kp);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->ki);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kd);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kf);

	p_sam_pack_t->data_lenght_u8 = pack_index_u16;
	p_sam_pack_t->pack_type_u8 = PID_SETTINGS_PACK;
}

void create_sam_ita_limits_pack (		SAM_Pack_t					*p_sam_pack_t,
								const 	Sam_ITA_Limits_Pack_t 		*sam_ita_limits_pack)
{
	UINT16 pack_index_u16 = 0;
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_ita_limits_pack->ita_min);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_ita_limits_pack->ita_max);
	FLOAT32_send(p_sam_pack_t->data_u8, &pack_index_u16, &sam_ita_limits_pack->ita_current);

	p_sam_pack_t->data_lenght_u8 = pack_index_u16;
	p_sam_pack_t->pack_type_u8 = ITA_LIMITS_PACK;
}

/*
 * solve_pack.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "solve_pack.h"

#include "pack_functions.h"


void solve_sam_command_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_Command_Pack_t	*sam_command_pack_t)
{
	UINT16 pack_index_u16 = 0;
	UINT8_get(p_sam_pack_t->data_u8, &pack_index_u16, &sam_command_pack_t->command);
	apply_commands();
}

void solve_sam_settings_pack( const SAM_Pack_t			*p_sam_pack_t,
									Sam_Settings_Pack_t	*sam_settings_pack_t)
{
	UINT16 pack_index_u16 = 0;
	UINT8_get  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_settings_pack_t->settings_id);
	UINT8_get  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_settings_pack_t->target_hand_id);
	UINT8_get  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_settings_pack_t->target_finger_id);
	UINT8_get  (p_sam_pack_t->data_u8, &pack_index_u16, &sam_settings_pack_t->target_joint_id);
	FLOAT32_get(p_sam_pack_t->data_u8, &pack_index_u16, &sam_settings_pack_t->data);
	apply_settings();
}

void solve_sam_pid_settings_pack( const SAM_Pack_t				*p_sam_pack_t,
										Sam_PID_Settings_Pack_t	*sam_pid_settings_pack_t)
{
	UINT16 pack_index_u16 = 0;

	UINT8_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->controller_id);
	UINT8_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->target_hand_id);
	UINT8_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->target_finger_id);
	UINT8_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->target_joint_id);
	FLOAT32_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kp);
	FLOAT32_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->ki);
	FLOAT32_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kd);
	FLOAT32_get	(p_sam_pack_t->data_u8, &pack_index_u16, &sam_pid_settings_pack_t->kf);

	update_pid_coef();
}

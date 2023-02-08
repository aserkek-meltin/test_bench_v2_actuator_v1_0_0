/*
 * solve_pack.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_SOLVE_PACK_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_SOLVE_PACK_H_

#include "comm_core/comm_functions.h"
#include "comm_core/sam_pro.h"
#include "comm_packs.h"
#include "../../utilities/Global.h"

void solve_sam_command_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_Command_Pack_t	*sam_command_pack_t);

void solve_sam_settings_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_Settings_Pack_t	*sam_settings_pack_t);

void solve_sam_pid_settings_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_PID_Settings_Pack_t	*sam_pid_settings_pack_t);

void solve_sam_gfs_lut_settings_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_LUT_Setting_Pack_t *sam_lut_setting_pack_t);

void solve_sam_ita_limits_pack( const 	SAM_Pack_t			*p_sam_pack_t,
									Sam_ITA_Limits_Pack_t *sam_ita_limits_pack_t);


#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_SOLVE_PACK_H_ */

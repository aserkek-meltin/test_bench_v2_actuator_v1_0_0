/*
 * create_pack.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_CREATE_PACK_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_CREATE_PACK_H_

#include "comm_core/comm_functions.h"
#include "comm_core/sam_pro.h"
#include "comm_packs.h"
#include "../../utilities/data_types.h"

void create_sam_status_pack(		SAM_Pack_t			*p_sam_pack_t,
							const 	Sam_Status_Pack_t 	*sam_status_pack_t);

void create_sam_update_pack(			SAM_Pack_t			*p_sam_pack_t,
							const 	Sam_Update_Pack_t 	*sam_update_pack_t);

void create_sam_pid_settings_pack(		SAM_Pack_t					*p_sam_pack_t,
								const 	Sam_PID_Settings_Pack_t 	*sam_pid_settings_pack_t);


#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_CREATE_PACK_H_ */

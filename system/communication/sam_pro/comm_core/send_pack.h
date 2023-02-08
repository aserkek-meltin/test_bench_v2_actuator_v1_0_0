/*
 * send_pack.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_SEND_PACK_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_SEND_PACK_H_

#include "sam_pro.h"
#include "../comm_packs.h"
#include "../create_pack.h"
#include "HardwareSerial.h"

void pack_send(	UINT8 * p_data_u8, UINT16 d_data_length, UINT32	bus);

void sam_pro_pack_send(const 	SAM_Pack_t * 	p_sam_pack_t,
								UINT32			bus);

void send_status_pack();
void send_test_pack();
void send_pid_coeff_pack(UINT8 _pid_id);
void send_ita_limits_pack();

#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_SEND_PACK_H_ */

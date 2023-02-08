/*
 * tasks.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_TASKS_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_TASKS_H_

#include "Arduino.h"
#include "../test_bench_v2_actuator_v1_0_0/system/utilities/loadcell_reader.h"

void task_system				( void * parameter );
void task_send_comm_pack		( void * parameter );
void task_loadcells_read		( void * parameter );
void task_right_hand			( void * parameter );
//void task_left_hand				( void * parameter );
void task_actuator_pos_send		( void * parameter );
void task_actuator_pos_read		( void * parameter );

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_TASKS_H_ */

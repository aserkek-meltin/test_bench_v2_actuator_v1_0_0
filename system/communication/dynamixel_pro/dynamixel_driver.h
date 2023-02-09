/*
 * dynamixel_driver.h
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_COMMUNICATION_DYNAMIXEL_PRO_DYNAMIXEL_DRIVER_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_COMMUNICATION_DYNAMIXEL_PRO_DYNAMIXEL_DRIVER_H_

#include "../../system_settings.h"
#include <array>

float pwm_pos2deg_pos(int pwm_pos);
int deg_pos2pwm_pos(float deg_pos);
bool dynamixel_initialization();
void ranges_initialization();
void torques_off();
void torques_on();
void dynamixel_sync_write(std::array<float, DXL_CNT> pwm_pos);
void dynamixel_sync_read();

#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_COMMUNICATION_DYNAMIXEL_PRO_DYNAMIXEL_DRIVER_H_ */

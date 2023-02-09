/*
 * loadcell_reader.h
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#ifndef TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_LOADCELL_READER_H_
#define TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_LOADCELL_READER_H_

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include <array>

#define I2C1_SDA_PIN 21		//NAU1
#define I2C1_SCL_PIN 22 	//NAU1
#define I2C2_SDA_PIN 4		//NAU2
#define I2C2_SCL_PIN 23		//NAU2


std::array<float, 2> read_sensors(uint8_t finger_id, uint8_t joint_id);
bool init_loadcells();


#endif /* TEST_BENCH_V2_ACTUATOR_V1_0_0_SYSTEM_UTILITIES_LOADCELL_READER_H_ */

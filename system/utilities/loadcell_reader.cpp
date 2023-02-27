/*
 * loadcell_reader.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "loadcell_reader.h"
#include "../system_settings.h"



#define NUMBER_OF_LOADCELLS	4

QWIICMUX i2c_mux;
NAU7802	**loadcells;
TwoWire I2C1 = TwoWire(0); 						//I2C1 bus

bool init_loadcells()
{
	bool couldnt_initialized = false;

	I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 100000); 	// AI2C
	loadcells = new NAU7802 *[NUMBER_OF_LOADCELLS];		// Create set of pointers to the class

	//Assign pointers to instances of the class
	for (int x = 0; x < NUMBER_OF_LOADCELLS; x++)
	{
		loadcells[x] = new NAU7802();
	}

	if (i2c_mux.begin(0x70, I2C1) == false)
	{
		//ERROR
		while (1);
	}
	//Serial.println("Mux detected");

	byte currentPortNumber = i2c_mux.getPort();
	//Serial.print("CurrentPort: ");
	//Serial.println(currentPortNumber);

	//Initialize all the sensors
	bool initSuccess = true;

	for (byte x = 0; x < NUMBER_OF_LOADCELLS; x++)
	{
		i2c_mux.setPort(x);
		if (loadcells[x]->begin(I2C1) == 0) //Begin returns 0 on a good init
		{
			//Serial.print("Sensor ");
			//Serial.print(x);
			//Serial.println(" did not begin! Check wiring");
			initSuccess = false;
		}
		else
		{
			//Configure each sensor
			//Serial.print("Sensor ");
			//Serial.print(x);
			//Serial.println(" configured");
		}
	}
}

std::array<float, 2> read_sensors(uint8_t hand_id, uint8_t finger_id, uint8_t joint_id)
{
	uint8_t its_address = (hand_id * 40) + (finger_id * 8) + (joint_id * 2);
	uint8_t gfs_address = its_address + 1;
	std::array<float,2> result; //array declared

	i2c_mux.setPort(its_address);
	result[0] = loadcells[its_address]->getReading();
	vTaskDelay(1 /portTICK_PERIOD_MS);		//5ms delay
	i2c_mux.setPort(gfs_address);
	result[1] = loadcells[gfs_address]->getReading();
	return result;
}

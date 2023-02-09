/*
 * loadcell_reader.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "loadcell_reader.h"
#include "../system_settings.h"


TwoWire I2C1 = TwoWire(0); 						//I2C1 bus
TwoWire I2C2 = TwoWire(1); 						//I2C2 bus
/*
NAU7802	its_loadcell	[NUMBER_OF_FINGERS]	[NUMBER_OF_TB];
NAU7802	gfs_loadcell	[NUMBER_OF_FINGERS]	[NUMBER_OF_TB];
*/
NAU7802 j1_its_loadcell;
NAU7802 j1_gfs_loadcell;
//NAU7802 j2_its_loadcell;
//NAU7802 j2_gfs_loadcell;

bool init_loadcells()
{
	I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 100000); // Start I2C1 on pins 21 and 22
	I2C2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 100000); // Start I2C2 on pins 0 and 23

	//Initialization of the loadcells
	bool is_initialized = true;
	/*
	for(int i=0; i<NUMBER_OF_FINGERS; i++)
	{
		for(int j=0; j<NUMBER_OF_TB; j++)
		{
			is_initialized = is_initialized & its_loadcell[0][i].begin(I2C1);
			is_initialized = is_initialized & gfs_loadcell[0][i].begin(I2C2);
		}
	}
*/
	j1_gfs_loadcell.begin(I2C1);
	j1_its_loadcell.begin(I2C2);

	if (is_initialized)
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::array<float, 2> read_sensors(uint8_t finger_id, uint8_t joint_id)
{
	std::array<float,2> result; //array declared
/*
	result[0] = its_loadcell[finger_id][joint_id].getReading();
	result[1] = gfs_loadcell[finger_id][joint_id].getReading();
*/
	result[0] = j1_its_loadcell.getReading();
	result[1] = j1_gfs_loadcell.getReading();
	return result;
}

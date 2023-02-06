/*
 * Global.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "Global.h"
#include "../system_settings.h"

Global::Global()
		: tb1(1,1,2,1)	//uint8_t _finger_id, uint8_t _test_bench_id, uint8_t _ita_id, uint8_t _jaa_id
		, tb2(1,2,4,3)	//uint8_t _finger_id, uint8_t _test_bench_id, uint8_t _ita_id, uint8_t _jaa_id
		, dxl(DXL_SERIAL, DXL_DIR_PIN)
{
	GL.GL_initialize();
}

void Global::GL_initialize()
{

}

float wrap360(float angle)
{
	if(angle < 0)
	{
		return wrap360(angle + 360);
	}
	else if(angle >= 360)
	{
		return wrap360(angle - 360);
	}
	else //degree >= 0 && degree < 360
	{
		return angle;
	}
}

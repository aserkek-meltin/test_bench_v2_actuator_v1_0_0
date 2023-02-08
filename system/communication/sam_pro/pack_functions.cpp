/*
 * pack_functions.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "pack_functions.h"

#include "comm_core/send_pack.h"
#include "comm_packs.h"
#include "../dynamixel_pro/dynamixel_driver.h"
//#include "../../../test_bench_actuator/controllers/internal_tension_control.h"
//#include "../../../test_bench_actuator/utilities/dynamixel.h"

void apply_commands()
{

		switch(GL.sam_command_pack_t.command)
		{
			case(SAM_CMD_ACTUATORS_TORQUES_OFF):
			{
				torques_off();

				GL.status_u.status_t.ACTUATOR_TORQUE = false;
				break;
			}
			case(SAM_CMD_ACTUATORS_TORQUES_ON):
			{
				torques_on();
				GL.status_u.status_t.ACTUATOR_TORQUE = true;
				break;
			}
			default:
				break;
		}
}

void apply_settings()
{
	switch(GL.sam_settings_pack_t.settings_id)
	{
		case(SAM_STG_IT_SETPOINT):
		{
			//GL.it_pid_settings_t.user_setpoint_SI = GL.sam_settings_pack_t.data;
			break;
		}
		default:
			break;
	}
}

void update_pid_coef()
{
	/*
	if(GL.sam_pid_settings_pack_t.pid_id == IT_PID)
	{
		xSemaphoreTake(GL.smp_it_pid_settings, portMAX_DELAY);
		GL.it_pid_settings_t.Kp = GL.sam_pid_settings_pack_t.kp;
		GL.it_pid_settings_t.Ki = GL.sam_pid_settings_pack_t.ki;
		GL.it_pid_settings_t.Kd = GL.sam_pid_settings_pack_t.kd;
		GL.it_pid.SetTunings(GL.it_pid_settings_t.Kp, GL.it_pid_settings_t.Ki, GL.it_pid_settings_t.Kd);
		xSemaphoreGive(GL.smp_it_pid_settings);
	}
	*/
}

void update_ita_limits()
{
	/*
	GL.ita_range_t.min = GL.sam_ita_limits_pack_t.ita_min;
	GL.ita_range_t.max = GL.sam_ita_limits_pack_t.ita_max;
	GL.ita_range_t.center = (GL.ita_range_t.min + GL.ita_range_t.max) / 2;
	*/
}

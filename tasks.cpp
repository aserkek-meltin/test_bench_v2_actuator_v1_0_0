/*
 * tasks.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "tasks.h"
#include "../test_bench_v2_actuator_v1_0_0/system/utilities/Global.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/sam_pro/comm_core/send_pack.h"


void task_system( void * parameter )
{
	int freq 			=  1;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	GL.system_counter_s++;
	GL.raise_command_recieved_flag_for_x_sec(1);
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_send_comm_pack( void * parameter )
{
	int freq 			=  50;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	send_status_pack();
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


void task_loadcells_read( void * parameter )
{
	int freq 			=  50;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	float *temp;
	temp = read_sensors(1, 1);
	GL.right_hand.index_finger.joint1.update_sensor_data(temp[0], temp[1]);

	temp = read_sensors(1, 2);
	GL.right_hand.index_finger.joint2.update_sensor_data(temp[0], temp[1]);
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_right_hand( void * parameter )
{
	int freq 			=  50;
    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	GL.right_hand.loop();
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


void task_actuator_pos_send( void * parameter )
{
	int freq 			=  50;
    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	//TODO - MOTOR - Motor SyncWrite Yazalim
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_actuator_pos_read( void * parameter )
{
	int freq 			=  50;
    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	//TODO - MOTOR - Motor SyncRead Yazalim
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

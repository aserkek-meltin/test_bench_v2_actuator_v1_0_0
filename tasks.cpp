/*
 * tasks.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "tasks.h"
#include "../test_bench_v2_actuator_v1_0_0/system/utilities/Global.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/sam_pro/comm_core/send_pack.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/dynamixel_pro/dynamixel_driver.h"


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
	int freq 			=  5;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	//send_status_pack();
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


void task_loadcells_read( void * parameter )
{
	int freq 			=  5;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	std::array<float, 2> result;
	result = read_sensors(1, 1);
	GL.right_hand.thumb_finger.joint1.update_sensor_data(result[0], result[1]);

	/*
	result = read_sensors(1, 2);
	Serial.println(result[0]);
	Serial.println(result[1]);
	Serial.println("--------------------");

	GL.right_hand.thumb_finger.joint2.update_sensor_data(result[0], result[1]);
	*/

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
	std::array<float, DXL_CNT> temp = GL.right_hand.get_dynamixel_commands();
	dynamixel_sync_write(temp);
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
	dynamixel_sync_read();
	Serial.print("1 JAA MCS: ");
	Serial.println(GL.right_hand.thumb_finger.joint1.get_jaa_mcs_angle());
	Serial.print("1 ITA: ");
	Serial.println(GL.right_hand.thumb_finger.joint1.get_ita_angle());

	Serial.print("2 JAA ECS: ");
	Serial.println(GL.right_hand.thumb_finger.joint2.get_jaa_mcs_angle());
	Serial.print("2 ITA: ");
	Serial.println(GL.right_hand.thumb_finger.joint2.get_ita_angle());


	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


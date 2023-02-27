/*
 * tasks.cpp
 *
 *  Created on: Feb 6, 2023
 *      Author: ErkekAbdul
 */

#include "tasks.h"
#include "../test_bench_v2_actuator_v1_0_0/system/utilities/Global.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/sam_pro/comm_core/send_pack.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/sam_pro/pack_functions.h"
#include "../test_bench_v2_actuator_v1_0_0/system/communication/dynamixel_pro/dynamixel_driver.h"
#include "../test_bench_v2_actuator_v1_0_0/system/utilities/ETK_communication.h"

void task_system( void * parameter )
{
	int freq 			=  1;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	GL.system_counter_s++;
	GL.raise_command_recieved_flag_for_x_sec(1);
	raise_flag_to_send_update_pack();
	Serial.println(GL.system_counter_s);
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_send_status_pack( void * parameter )
{
	int freq 			=  10;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	send_status_pack();			//5647 microseconds. = 6 miliseconds
	//TASK CODE ABOVE
	t_end = millis();
	if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
	else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_send_update_pack( void * parameter )
{
	int freq 			=  5;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	if(GL.is_uptade_needed)
	{
		send_update_pack();
	}
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}

void task_read_comm( void * parameter )
{
	int freq 			=  5;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	//TODO -Communication - Read from SAM PRO
	//read_Serial_port();
	read_FM_UARTPort();
	read_serial_communication();
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


void task_loadcells_read( void * parameter )
{
	int freq 			=  100;

    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = micros();

	//TASK CODE BELOW
	//1 joint (2 loadcells) takes around 4 miliseconds to read
	std::array<float, 2> result;
	result = read_sensors(0, 0, 0);
	GL.right_hand.thumb_finger.joint1.update_sensor_data(result[0], result[1]);

	result = read_sensors(0, 0, 1);
	GL.right_hand.thumb_finger.joint2.update_sensor_data(result[0], result[1]);
	//TASK CODE ABOVE

	t_end = micros ();
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
	int freq 			=  100;
    int period 	= (1000 / freq); long t_start = 0; long t_end = 0;
	for (;;){t_start = millis();

	//TASK CODE BELOW
	if(GL.status_u.status_t.ACTUATOR_TORQUE == true)
	{
		xSemaphoreTake(GL.smp_dxl_communication, portMAX_DELAY);
		std::array<float, DXL_CNT> temp = GL.right_hand.get_dynamixel_commands();
		dynamixel_sync_write(temp);
		xSemaphoreGive(GL.smp_dxl_communication);
	}


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
	/*
	Serial.print("1 JAA MCS: ");
	Serial.println(GL.right_hand.thumb_finger.joint1.get_jaa_mcs_angle());
	Serial.print("1 ITA: ");
	Serial.println(GL.right_hand.thumb_finger.joint1.get_ita_angle());

	Serial.print("2 JAA ECS: ");
	Serial.println(GL.right_hand.thumb_finger.joint2.get_jaa_mcs_angle());
	Serial.print("2 ITA: ");
	Serial.println(GL.right_hand.thumb_finger.joint2.get_ita_angle());
*/
	//TASK CODE ABOVE

		t_end = millis();
		if((t_end -t_start) > period && (t_end - t_start) > 0){} //Error
		else { vTaskDelay(period - (t_end -t_start) / portTICK_PERIOD_MS); }
	}
}


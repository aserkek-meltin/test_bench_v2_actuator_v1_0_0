/*
 * dynamixel_driver.cpp
 *
 *  Created on: Feb 8, 2023
 *      Author: ErkekAbdul
 */

#include "dynamixel_driver.h"
#include "../../utilities/Global.h"
#include "../../system_settings.h"

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;

const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_CNT];

sw_data_t sw_data[DXL_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_CNT];

using namespace ControlTableItem;

float pwm_pos2deg_pos(int pwm_pos)
{
	return float(pwm_pos * 360.0 / 4069.0);
}

int deg_pos2pwm_pos(float deg_pos)
{
	return (int)(deg_pos / 360 * 4069);
}

bool dynamixel_initialization()
{
	bool is_initialized = true;
	GL.dxl.begin(4000000);
	GL.dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_1_JAA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_1_ITA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_2_JAA_ID);
	is_initialized = is_initialized & GL.dxl.ping(FINGER_1_JOINT_2_ITA_ID);

	ranges_initialization();
	return is_initialized;
}

void ranges_initialization()
{
	//TODO - FUTURE - Can take ranges from EEPROM.
	//Initialize the ranges of the Joints
	Range_t ita;
	Range_t jaa_mcs;

	//Finger1 - Joint1
	ita.min = FINGER_1_JOINT_1_ITA_MIN_RAW;
	ita.max = FINGER_1_JOINT_1_ITA_MAX_RAW;
	ita.center = (ita.min + ita.max)/2;
	jaa_mcs.min = FINGER_1_JOINT_1_JAA_MIN_MCS_DEG;
	jaa_mcs.max = FINGER_1_JOINT_1_JAA_MAX_MCS_DEG;
	jaa_mcs.center = (jaa_mcs.min + jaa_mcs.max)/2;
	GL.right_hand.thumb_finger.joint1.update_ranges(ita, jaa_mcs);

	//Finger2 - Joint2
	ita.min = FINGER_1_JOINT_2_ITA_MIN_RAW;
	ita.max = FINGER_1_JOINT_2_ITA_MAX_RAW;
	ita.center = (ita.min + ita.max)/2;
	jaa_mcs.min = FINGER_1_JOINT_2_JAA_MIN_MCS_DEG;
	jaa_mcs.max = FINGER_1_JOINT_2_JAA_MAX_MCS_DEG;
	jaa_mcs.center = wrap360(jaa_mcs.min + jaa_mcs.max)/2;
	GL.right_hand.thumb_finger.joint2.update_ranges(ita, jaa_mcs);
}

void torques_off()
{

}

void torques_on()
{

}


void dynamixel_sync_write(std::array<float, DXL_CNT> pwm_pos)
{
	//TODO - MOTOR - Motor SyncWrite Yazalim

	// Fill the members of structure to syncWrite using internal packet buffer
	sw_infos.packet.p_buf = nullptr;
	sw_infos.packet.is_completed = false;
	sw_infos.addr = SW_START_ADDR;
	sw_infos.addr_length = SW_ADDR_LEN;
	sw_infos.p_xels = info_xels_sw;
	sw_infos.xel_count = 0;

	for(int i = 0; i < DXL_CNT; i++)
	{
		info_xels_sw[i].id = DXL_LIST[i];
		info_xels_sw[i].p_data = (UINT8*)&sw_data[i].goal_position;
		sw_infos.xel_count++;
	}
	sw_infos.is_info_changed = true;


	//Inserting new Goal Positions
	if(GL.right_hand.thumb_finger.joint1.is_controller_on())
	{
		sw_data[0].goal_position = pwm_pos[0];
		sw_data[1].goal_position = pwm_pos[1];
	}
	if(GL.right_hand.thumb_finger.joint2.is_controller_on())
	{
		sw_data[2].goal_position = pwm_pos[2];
		sw_data[3].goal_position = pwm_pos[3];
	}


	// Update the SyncWrite packet status
	sw_infos.is_info_changed = true;
	// Build a SyncWrite Packet and transmit to DYNAMIXEL
	if(GL.dxl.syncWrite(&sw_infos) == true)
	{
		//Success
		for(int i = 0; i<sw_infos.xel_count; i++)
		{
			//Serial.print("  ID: ");					Serial.println(sw_infos.p_xels[i].id);
			//Serial.print("\t Goal Position: ");		Serial.println(sw_data[i].goal_position);
		}
	}
	else
	{
		//FAIL
		//Serial.print("[SyncWrite] Fail, Lib error code: ");
		//Serial.println(GL.dxl.getLastLibErrCode());
	}

}

void dynamixel_sync_read()
{
	// Fill the members of structure to syncRead using external user packet buffer
	sr_infos.packet.p_buf = user_pkt_buf;
	sr_infos.packet.buf_capacity = user_pkt_buf_cap;
	sr_infos.packet.is_completed = false;
	sr_infos.addr = SR_START_ADDR;
	sr_infos.addr_length = SR_ADDR_LEN;
	sr_infos.p_xels = info_xels_sr;
	sr_infos.xel_count = 0;

	for(int i = 0; i < DXL_CNT; i++)
	{
		info_xels_sr[i].id = DXL_LIST[i];
		info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
		sr_infos.xel_count++;
	}
	sr_infos.is_info_changed = true;

	// Transmit predefined SyncRead instruction packet
	// and receive a status packet from each DYNAMIXEL
	xSemaphoreTake(GL.smp_dxl_communication, portMAX_DELAY);
	uint8_t recv_cnt = GL.dxl.syncRead(&sr_infos);
	xSemaphoreGive(GL.smp_dxl_communication);
	if(recv_cnt > 0)
	{
		GL.right_hand.thumb_finger.joint1.jaa_set_mcs_angle_readings	(pwm_pos2deg_pos(sr_data[0].present_position));
		GL.right_hand.thumb_finger.joint1.ita_set_angle_readings		(sr_data[1].present_position);
		GL.right_hand.thumb_finger.joint2.jaa_set_mcs_angle_readings	(pwm_pos2deg_pos(sr_data[2].present_position));
		GL.right_hand.thumb_finger.joint2.ita_set_angle_readings		(sr_data[3].present_position);
	}
}


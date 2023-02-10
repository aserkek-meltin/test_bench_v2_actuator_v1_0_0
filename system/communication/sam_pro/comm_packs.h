/*
 * communication_packs.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_


#include "comm_core/sam_pro.h"
#include "../../utilities/data_types.h"

typedef enum
{
    SAM_CMD_ACTUATORS_TORQUES_OFF = 0,
    SAM_CMD_ACTUATORS_TORQUES_ON,
    SAM_CMD_BWBS_CALIBRATE,
    SAM_CMD_SENSORS_CALIBRATE,
}commands;

typedef enum
{
    SAM_STG_ITA_CALIBRATE = 0,
    SAM_STG_ITA_POS_uM_SP,
    SAM_STG_JAA_POS_uDEG_SP,
    SAM_STG_JOINT_TORQUE_SP_mN,
    SAM_STG_GET_PIDF_COEFFICIENTS,
    SAM_STG_JOINT_TORQUE_CONTROLLER_SWITCH,
}setting_ids;

typedef enum
{
	IT1_PID,
	IT2_PID,
}pid_ids;

typedef enum
{
	COMMAND_PACK,
	SETTINGS_PACK,
	PID_SETTINGS_PACK,
	STATUS_PACK,
	UPDATE_PACK,
}pack_types;

typedef struct
{
	UINT8	command;					//Command IDA
}Sam_Command_Pack_t;

typedef struct
{
	UINT8		settings_id;				//Setting ID
	UINT8		target_hand_id;
	UINT8		target_finger_id;
	UINT8		target_joint_id;
	FLOAT32		data;						//Value for the setting
}Sam_Settings_Pack_t;


typedef struct
{
	UINT32	system_counter_s;			//Counter that increases every 1 sec

	UINT8	errors;						//Shows the errors in the system
	UINT8	status;						//Shows the status of the system components

	FLOAT32	joint_angle_est;			//Joint Angle Estimation
	FLOAT32 joint_jaa_angle;			//JAA Angle

	FLOAT32 joint_ita_mm;				//ITA pos to mm
	FLOAT32 joint_it1_est;				//IT1 estimation [N]
	FLOAT32 joint_it1_sp;				//IT1 setpoint   [N]
	FLOAT32 joint_it2_est;				//IT2 estimation [N]
	FLOAT32 joint_it2_sp;				//IT2 setpoint   [N]]

	FLOAT32 test_value0_f32;
	FLOAT32 test_value1_f32;
	FLOAT32 test_value2_f32;
	FLOAT32 test_value3_f32;
	FLOAT32 test_value4_f32;

	UINT8 finger_id;
	UINT8 joint_id;

}Sam_Status_Pack_t;

typedef struct
{
	UINT32	test_value1_u32;

	UINT8	test_bits1;
	UINT8	test_bits2;

	FLOAT32	joint_jaa_min		;
	FLOAT32 joint_jaa_max		;
	FLOAT32 joint_ita_min		;
	FLOAT32 joint_ita_max		;
	FLOAT32 joint_torque_sp		;
	FLOAT32 test_value6_f32		;
	FLOAT32 test_value7_f32		;
	FLOAT32 test_value8_f32		;
	FLOAT32 test_value9_f32		;
	FLOAT32 test_value10_f32	;
	FLOAT32 test_value11_f32	;
	FLOAT32 test_value12_f32	;

	UINT8 finger_id;
	UINT8 joint_id;

}Sam_Update_Pack_t;


typedef struct
{
	UINT8		controller_id;
	UINT8		target_hand_id;
	UINT8		target_finger_id;
	UINT8		target_joint_id;
	FLOAT32   	kp;
	FLOAT32   	ki;
	FLOAT32   	kd;
	FLOAT32   	kf;
}Sam_PID_Settings_Pack_t;

#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_PACKS_H_ */

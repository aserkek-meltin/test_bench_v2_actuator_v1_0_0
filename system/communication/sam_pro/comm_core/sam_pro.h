/*
 * sam_pro.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_SAM_PRO_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_SAM_PRO_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//INCLUDES---------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef 	signed 		char 	INT8;
typedef 	signed 		char 	SINT8;
typedef 	unsigned 	char 	UINT8;
//typedef		byte	UINT8;

typedef 	signed 		short 	INT16;
typedef 	signed 		short 	SINT16;
typedef 	unsigned	short	UINT16;

typedef 	signed 		int 	INT32;
typedef 	signed 		int 	SINT32;
typedef 	unsigned 	int 	UINT32;

typedef 	signed 		long long 	INT64;
typedef 	signed 		long long 	SINT64;
typedef 	unsigned 	long long	UINT64;

typedef 	float					FLOAT32;
typedef		double					FLOAT64;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINES----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SYNCRON_1 			0X34
#define SYNCRON_2 			0X35
#define SAM_PACK_HEADER 	4
#define SAM_PACK_OVERAGE 	6
#define SAM_PACK_MAX_LENGTH 255

#define HIGH_MASK			0XFF00
#define LOW_MASK		 	0X00FF

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//STRUCTS----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
	UINT8 syncron1_u8;
	UINT8 syncron2_u8;
	UINT8 pack_type_u8;
	UINT8 data_lenght_u8;
	UINT8 data_u8[SAM_PACK_MAX_LENGTH];
	UINT8 CRC_MSByte_u8;
	UINT8 CRC_LSByte_u8;
} SAM_Pack_t;

typedef struct {
	UINT8 pack_catch_state;
	UINT8 data_counter;
	SAM_Pack_t SAM_Pack;
	void (*solve_pack)(const SAM_Pack_t* p_captured_pack);

	UINT32	incoming_byte_number_u32;
	UINT32	last_byte_number_u32;
	UINT32	captured_pack_number_u32;
	UINT32	last_pack_number_u32;
	UINT32	last_crc_error_number_u32;
} SAM_Channel_t;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VARIABLES--------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OBJECTS----------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS--------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Catch_Data_From_Bytes(	SAM_Channel_t 	p_channel,
							const UINT8*	captured_data,
							const UINT32	captured_data_lenght);

void Pack_It_SAM		  ( SAM_Pack_t* p_sam_pack);

void Reset_SAM_Channel ( SAM_Channel_t * p_sam_channel,
						void (*pack_solve)(const SAM_Pack_t*));

#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_SAM_PRO_H_ */

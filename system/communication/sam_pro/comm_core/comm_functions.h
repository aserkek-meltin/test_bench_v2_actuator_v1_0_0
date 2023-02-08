/*
 * comm_functions.h
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#ifndef TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_CORE_COMM_FUNCTIONS_H_
#define TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_CORE_COMM_FUNCTIONS_H_

#include "../../../utilities/data_types.h"

void INT8_send(				UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
				const 		INT8* 		p_data_s8					);

void UINT8_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT8* 		p_data_u8					);

void INT16_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT16* 		p_data_s16					);

void UINT16_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT16*		p_data_u16					);

void INT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT32* 		p_data_s32					);

void UINT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT32* 	p_data_u32					);

void UINT32_send2(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT32* 	p_data_u32					);

void INT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT64* 		p_data_s64					);

void UINT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT64* 	p_data_u64					);

void FLOAT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	FLOAT32* 	p_data_f32					);

void FLOAT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	FLOAT64* 	p_data_f64					);

void INT8_get( 		const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							INT8*		p_data_s8						);

void UINT8_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							UINT8*		p_data_u8						);

void INT16_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							INT8*		p_data_s16						);

void UINT16_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							UINT16*		p_data_u16						);

void INT32_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							INT32*		p_data_s32						);

void UINT32_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							UINT32*		p_data_u32						);

void INT64_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							INT64*		p_data_s64						);


void UINT64_get(  	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							UINT64*		p_data_u64						);

void FLOAT32_get( 	const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							FLOAT32*	p_data_f32						);

void FLOAT64_get(  const 	UINT8* 		p_pack_u8						,
							UINT16*		p_index_u16						,
							FLOAT64*	p_data_f64						);


#endif /* TEST_BENCH_ACTUATOR_COMMUNICATION_COMM_CORE_COMM_FUNCTIONS_H_ */

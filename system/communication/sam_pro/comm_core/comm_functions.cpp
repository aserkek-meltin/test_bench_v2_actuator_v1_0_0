/*
 * comm_functions.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "comm_functions.h"

#include "Arduino.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BYTE SEND FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT8_send(				UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
				const 		INT8* 		p_data_s8					)
{
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_s8 )[0]	;

	*p_index_u16					= *p_index_u16 + sizeof(INT8)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT8_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT8* 		p_data_u8					)
{
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_u8 )[0]	;
	*p_index_u16					= *p_index_u16 + sizeof(UINT8)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT16_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT16* 		p_data_s16					)
{
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_s16 )[0]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_s16 )[1]	;

	*p_index_u16					= *p_index_u16 + sizeof(INT16)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT16_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT16*		p_data_u16					)
{
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_u16 )[0]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_u16 )[1]	;

	*p_index_u16					= *p_index_u16 + sizeof(UINT16)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT32* 		p_data_s32					)
{
	p_pack_u8[ (*p_index_u16) + 3 ] = ( ( UINT8 *) p_data_s32 )[0]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = ( ( UINT8 *) p_data_s32 )[1]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_s32 )[2]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_s32 )[3]	;

	*p_index_u16					= *p_index_u16 + sizeof(INT32)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT32_send2(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT32* 	p_data_u32					)
{
	p_pack_u8[ (*p_index_u16) + 3 ] = ( ( UINT8 *) p_data_u32 )[0]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = ( ( UINT8 *) p_data_u32 )[1]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_u32 )[2]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_u32 )[3]	;

	*p_index_u16					= *p_index_u16 + sizeof(UINT32)	;
}

void UINT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT32* 	p_data_u32					)
{
	union
	{
		uint32_t u32_variable;
		byte temp_array[4];
	}u;
	u.u32_variable = *p_data_u32;

	p_pack_u8[ (*p_index_u16) + 3 ] = u.temp_array[3]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = u.temp_array[2]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = u.temp_array[1]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = u.temp_array[0]	;
	*p_index_u16					= *p_index_u16 + sizeof(UINT32);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	INT64* 		p_data_s64					)
{
	p_pack_u8[ (*p_index_u16) + 7 ] = ( ( UINT8 *) p_data_s64 )[0]	;
	p_pack_u8[ (*p_index_u16) + 6 ] = ( ( UINT8 *) p_data_s64 )[1]	;
	p_pack_u8[ (*p_index_u16) + 5 ] = ( ( UINT8 *) p_data_s64 )[2]	;
	p_pack_u8[ (*p_index_u16) + 4 ] = ( ( UINT8 *) p_data_s64 )[3]	;
	p_pack_u8[ (*p_index_u16) + 3 ] = ( ( UINT8 *) p_data_s64 )[4]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = ( ( UINT8 *) p_data_s64 )[5]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_s64 )[6]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_s64 )[7]	;

	*p_index_u16					= *p_index_u16 + sizeof(INT64)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	UINT64* 	p_data_u64					)
{
	p_pack_u8[ (*p_index_u16) + 7 ] = ( ( UINT8 *) p_data_u64 )[0]	;
	p_pack_u8[ (*p_index_u16) + 6 ] = ( ( UINT8 *) p_data_u64 )[1]	;
	p_pack_u8[ (*p_index_u16) + 5 ] = ( ( UINT8 *) p_data_u64 )[2]	;
	p_pack_u8[ (*p_index_u16) + 4 ] = ( ( UINT8 *) p_data_u64 )[3]	;
	p_pack_u8[ (*p_index_u16) + 3 ] = ( ( UINT8 *) p_data_u64 )[4]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = ( ( UINT8 *) p_data_u64 )[5]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_u64 )[6]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_u64 )[7]	;

	*p_index_u16					= *p_index_u16 + sizeof(UINT64)	;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FLOAT32_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	FLOAT32* 	p_data_f32					)
{
	union
	{
		float float_variable;
		byte temp_array[4];
	}u;
	u.float_variable = *p_data_f32;

	p_pack_u8[ (*p_index_u16) + 3 ] = u.temp_array[3]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = u.temp_array[2]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = u.temp_array[1]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = u.temp_array[0]	;
	*p_index_u16					= *p_index_u16 + sizeof(FLOAT32);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FLOAT64_send(			UINT8*		p_pack_u8					,
							UINT16* 	p_index_u16					,
					const 	FLOAT64* 	p_data_f64					)
{
	p_pack_u8[ (*p_index_u16) + 7 ] = ( ( UINT8 *) p_data_f64 )[0]	;
	p_pack_u8[ (*p_index_u16) + 6 ] = ( ( UINT8 *) p_data_f64 )[1]	;
	p_pack_u8[ (*p_index_u16) + 5 ] = ( ( UINT8 *) p_data_f64 )[2]	;
	p_pack_u8[ (*p_index_u16) + 4 ] = ( ( UINT8 *) p_data_f64 )[3]	;
	p_pack_u8[ (*p_index_u16) + 3 ] = ( ( UINT8 *) p_data_f64 )[4]	;
	p_pack_u8[ (*p_index_u16) + 2 ] = ( ( UINT8 *) p_data_f64 )[5]	;
	p_pack_u8[ (*p_index_u16) + 1 ] = ( ( UINT8 *) p_data_f64 )[6]	;
	p_pack_u8[ (*p_index_u16) + 0 ] = ( ( UINT8 *) p_data_f64 )[7]	;

	*p_index_u16					= *p_index_u16 + sizeof(FLOAT64);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////  |
// BYTE GET FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void INT8_get( 		const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							INT8*	p_data_s8						)
{
	( (UINT8*) p_data_s8) [0] 	= p_pack_u8[ (*p_index_u16) + 0];

	*p_index_u16 				= *p_index_u16 + sizeof(INT8);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT8_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							UINT8*	p_data_u8						)
{
	( (UINT8*) p_data_u8) [0] 	= p_pack_u8[ (*p_index_u16) + 0];

	*p_index_u16 				= *p_index_u16 + sizeof(UINT8);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT16_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							INT8*	p_data_s16						)
{
	( (UINT8*) p_data_s16) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_s16) [1] 	= p_pack_u8[ (*p_index_u16) + 1];

	*p_index_u16 				= *p_index_u16 + sizeof(INT16);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT16_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							UINT16*	p_data_u16						)
{
	( (UINT8*) p_data_u16) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_u16) [1] 	= p_pack_u8[ (*p_index_u16) + 1];

	*p_index_u16 				= *p_index_u16 + sizeof(UINT16);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT32_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							INT32*	p_data_s32						)
{
	( (UINT8*) p_data_s32) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_s32) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_s32) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_s32) [3] 	= p_pack_u8[ (*p_index_u16) + 3];

	*p_index_u16 				= *p_index_u16 + sizeof(INT32);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT32_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							UINT32*	p_data_u32						)
{
	( (UINT8*) p_data_u32) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_u32) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_u32) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_u32) [3] 	= p_pack_u8[ (*p_index_u16) + 3];

	*p_index_u16 				= *p_index_u16 + sizeof(UINT32);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void INT64_get( 	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							INT64*	p_data_s64						)
{
	( (UINT8*) p_data_s64) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_s64) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_s64) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_s64) [3] 	= p_pack_u8[ (*p_index_u16) + 3];
	( (UINT8*) p_data_s64) [4] 	= p_pack_u8[ (*p_index_u16) + 4];
	( (UINT8*) p_data_s64) [5] 	= p_pack_u8[ (*p_index_u16) + 5];
	( (UINT8*) p_data_s64) [6] 	= p_pack_u8[ (*p_index_u16) + 6];
	( (UINT8*) p_data_s64) [7] 	= p_pack_u8[ (*p_index_u16) + 7];

	*p_index_u16 				= *p_index_u16 + sizeof(INT64);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UINT64_get(  	const 	UINT8* 	p_pack_u8						,
							UINT16*	p_index_u16						,
							UINT64*	p_data_u64						)
{
	( (UINT8*) p_data_u64) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_u64) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_u64) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_u64) [3] 	= p_pack_u8[ (*p_index_u16) + 3];
	( (UINT8*) p_data_u64) [4] 	= p_pack_u8[ (*p_index_u16) + 4];
	( (UINT8*) p_data_u64) [5] 	= p_pack_u8[ (*p_index_u16) + 5];
	( (UINT8*) p_data_u64) [6] 	= p_pack_u8[ (*p_index_u16) + 6];
	( (UINT8*) p_data_u64) [7] 	= p_pack_u8[ (*p_index_u16) + 7];

	*p_index_u16 				= *p_index_u16 + sizeof(UINT64);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FLOAT32_get( 	const 	UINT8* p_pack_u8						,
							UINT16*	p_index_u16						,
							FLOAT32*p_data_f32						)
{
	( (UINT8*) p_data_f32) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_f32) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_f32) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_f32) [3] 	= p_pack_u8[ (*p_index_u16) + 3];

	*p_index_u16 				= *p_index_u16 + sizeof(FLOAT32);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FLOAT64_get(  const 	UINT8* p_pack_u8						,
							UINT16*	p_index_u16						,
							FLOAT64*p_data_f64						)
{
	( (UINT8*) p_data_f64) [0] 	= p_pack_u8[ (*p_index_u16) + 0];
	( (UINT8*) p_data_f64) [1] 	= p_pack_u8[ (*p_index_u16) + 1];
	( (UINT8*) p_data_f64) [2] 	= p_pack_u8[ (*p_index_u16) + 2];
	( (UINT8*) p_data_f64) [3] 	= p_pack_u8[ (*p_index_u16) + 3];
	( (UINT8*) p_data_f64) [4] 	= p_pack_u8[ (*p_index_u16) + 4];
	( (UINT8*) p_data_f64) [5] 	= p_pack_u8[ (*p_index_u16) + 5];
	( (UINT8*) p_data_f64) [6] 	= p_pack_u8[ (*p_index_u16) + 6];
	( (UINT8*) p_data_f64) [7] 	= p_pack_u8[ (*p_index_u16) + 7];

	*p_index_u16 				= *p_index_u16 + sizeof(FLOAT64);
}

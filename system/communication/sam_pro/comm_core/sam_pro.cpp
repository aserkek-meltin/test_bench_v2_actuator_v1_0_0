/*
 * sam_pro.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "sam_pro.h"
#include "recieve_pack.h"
#include "Arduino.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ENUMS------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum PACK_CATCH_STATES {
	SYNCHRON_1_CATCH_SAM = 0,
	SYNCHRON_2_CATCH_SAM,
	PACK_TYPE_CATCH_SAM,
	PACK_LENGTH_CATCH_SAM,
	DATA_CATCH_SAM,
	CRC_1_CATCH_SAM,
	CRC_2_CATCH_SAM
};


void Catch_Data_From_Bytes(	SAM_Channel_t 	p_channel,
							const UINT8*	captured_data,
							const UINT32	captured_data_lenght)
{
	INT32 i = 0;
	for(i=0; i< captured_data_lenght; i++){
		p_channel.incoming_byte_number_u32++;

		switch( p_channel.pack_catch_state){
		case SYNCHRON_1_CATCH_SAM:
		{
			if((byte)captured_data[i] == (byte)SYNCRON_1){
				p_channel.SAM_Pack.syncron1_u8 = captured_data[i];
				p_channel.pack_catch_state = SYNCHRON_2_CATCH_SAM;
			}
			break;
		}
		case SYNCHRON_2_CATCH_SAM:
		{
			if((byte)captured_data[i] == (byte)SYNCRON_2){
				p_channel.SAM_Pack.syncron2_u8 = captured_data[i];
				p_channel.pack_catch_state = PACK_TYPE_CATCH_SAM;
			}
			else
			{
				p_channel.pack_catch_state = SYNCHRON_1_CATCH_SAM;
				p_channel.data_counter = 0;
			}

			break;
		}
		case PACK_TYPE_CATCH_SAM:
		{
			p_channel.SAM_Pack.pack_type_u8 = captured_data[i];
			p_channel.pack_catch_state = PACK_LENGTH_CATCH_SAM;
			break;
		}
		case PACK_LENGTH_CATCH_SAM:
		{
			p_channel.SAM_Pack.data_lenght_u8 = captured_data[i];
			if(p_channel.SAM_Pack.data_lenght_u8 == 0){
				p_channel.pack_catch_state = CRC_1_CATCH_SAM;
			}
			else{
				p_channel.pack_catch_state = DATA_CATCH_SAM;
			}
			break;
		}
		case DATA_CATCH_SAM:
		{
			p_channel.SAM_Pack.data_u8[p_channel.data_counter] = captured_data[i];
			p_channel.data_counter++;

			if(p_channel.data_counter >= p_channel.SAM_Pack.data_lenght_u8){
				p_channel.pack_catch_state = CRC_1_CATCH_SAM;
			}
			break;
		}
		case CRC_1_CATCH_SAM:
		{
			p_channel.SAM_Pack.CRC_MSByte_u8 = captured_data[i];
			p_channel.pack_catch_state = CRC_2_CATCH_SAM;
			break;
		}
		case CRC_2_CATCH_SAM:
		{
			UINT16 data_pack_length_u16			= 0;
			UINT16 crc_calculated				= 0;
			UINT16 crc_incoming					= 0;

			p_channel.SAM_Pack.CRC_LSByte_u8 = captured_data[i];
			data_pack_length_u16 = p_channel.SAM_Pack.data_lenght_u8;

			crc_calculated = 39304; //CRC16((UINT8 *) &p_channel.SAM_Pack), data_pack_length_u16 + SAM_PACK_HEADER);

			crc_incoming = (p_channel.SAM_Pack.CRC_MSByte_u8 <<8) + (p_channel.SAM_Pack.CRC_LSByte_u8);

			if(crc_calculated == crc_incoming)
			{
				p_channel.solve_pack(&p_channel.SAM_Pack);
				//p_channel.captured_pack_number_u32++;

				p_channel.pack_catch_state = SYNCHRON_1_CATCH_SAM;
				p_channel.data_counter = 0;
			}
			else{
				//Wrong CRC
				p_channel.pack_catch_state = SYNCHRON_1_CATCH_SAM;
				p_channel.data_counter = 0;
			}
			break;
		}
		default:
		{
			break;
		}


		}
	}
}

void Reset_SAM_Channel ( SAM_Channel_t * p_sam_channel,
						void (*pack_solve)(const SAM_Pack_t*))
{
	p_sam_channel->pack_catch_state = 0;
	p_sam_channel->data_counter = 0;
	p_sam_channel->captured_pack_number_u32 = 0;
	p_sam_channel->last_crc_error_number_u32 = 0;
	p_sam_channel->solve_pack = solve_pack;
}

void Pack_It_SAM		  ( SAM_Pack_t* p_sam_pack)
{
	UINT16	calculated_CRC_u16 = 0;

	p_sam_pack->syncron1_u8 = SYNCRON_1;
	p_sam_pack->syncron2_u8 = SYNCRON_2;

	/*
	calculated_CRC_u16 = CRC16((UINT8*)p_sam_pack,
								p_sam_pack.data_lenght_u8,
								SAM_PACK_HEADER			  );
	*/

	calculated_CRC_u16  = 0x9988;

	p_sam_pack->CRC_MSByte_u8 = (UINT16) ((calculated_CRC_u16 & HIGH_MASK) >> 8)  ;
	p_sam_pack->CRC_LSByte_u8 = (UINT16)  (calculated_CRC_u16 & LOW_MASK) 		  ;

	p_sam_pack->CRC_MSByte_u8 = (UINT16) ((calculated_CRC_u16 & HIGH_MASK) >> 8)  ;
	p_sam_pack->CRC_LSByte_u8 = (UINT16)  (calculated_CRC_u16 & LOW_MASK) 		  ;
}

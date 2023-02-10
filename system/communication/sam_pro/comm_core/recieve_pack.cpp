/*
 * recieve_pack.cpp
 *
 *  Created on: 1 Eki 2022
 *      Author: sam
 */

#include "recieve_pack.h"
#include "../pack_functions.h"

void solve_pack(const SAM_Pack_t *p_incoming_sam_t)
{
	//GL.last_pack_time = now
	//toggle_flag();
	GL.status_u.status_t.COMMAND_RECIEVED 	= true;
	GL.temp_command_recieved_counter 		= 0;
	switch(p_incoming_sam_t->pack_type_u8)
	{
		GL.temp_command_recieved_counter 		= 0;
		case COMMAND_PACK:
		{
			solve_sam_command_pack(p_incoming_sam_t, &GL.sam_command_pack_t);
			break;
		}
		case SETTINGS_PACK:
		{
			solve_sam_settings_pack(p_incoming_sam_t, &GL.sam_settings_pack_t);
			break;
		}
		case PID_SETTINGS_PACK:
		{
			solve_sam_pid_settings_pack(p_incoming_sam_t, &GL.sam_pid_settings_pack_t);
			break;
		}
	default:
		break;
	}

}

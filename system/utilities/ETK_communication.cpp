/*
 * ETK_communication.cpp
 *
 *  Created on: Feb 27, 2023
 *      Author: ErkekAbdul
 */

#include "ETK_communication.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VARIABLES--------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool new_data = false;
const int numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];
int lost_pack_number;

void recieve_pack()
{
    static bool recvInProgress = false;
    static int ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (GL.fm_UARTPort.available() > 0 && new_data == false) {
        rc = GL.fm_UARTPort.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                new_data = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parse_pack()
{
	Communication_pack_t			incoming_pack_t;

    char * strtokIndx; // this is used by strtok() as an index
    float acceptedDiff = 0.01;
    incoming_pack_t.is_pack_healthy = false;

    strtokIndx = strtok(tempChars, ",");
    incoming_pack_t.ja_1 = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    incoming_pack_t.ja_2 = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    incoming_pack_t.correction = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    incoming_pack_t.pack_number = atoi(strtokIndx);
    if(fabsf(incoming_pack_t.correction - 34.34) < acceptedDiff){
    	incoming_pack_t.is_pack_healthy = true;
    	GL.communication_pack_t.ja_1             = incoming_pack_t.ja_1;
		GL.communication_pack_t.ja_2             = incoming_pack_t.ja_2;
		GL.communication_pack_t.correction       = incoming_pack_t.correction;
		GL.communication_pack_t.is_pack_healthy  = incoming_pack_t.is_pack_healthy;
		GL.communication_pack_t.pack_number      = incoming_pack_t.pack_number;
	}
    else{
    	incoming_pack_t.is_pack_healthy = false;
    	lost_pack_number++;
    }
}

void show_pack() {
	#ifndef PLOTMODE
	Serial.println("------------------------------------------------------");
    Serial.print("Data 1: ");    		Serial.println(GL.communication_pack_t.ja_1);
    Serial.print("Data 2: ");    		Serial.println(GL.communication_pack_t.ja_2);
    Serial.print("Correction 1: ");    	Serial.println(GL.communication_pack_t.correction);
    Serial.print("Pack Number: ");    	Serial.println(GL.communication_pack_t.pack_number);
    Serial.print("Is Pack Healthy: ");  Serial.println(GL.communication_pack_t.is_pack_healthy);
    Serial.print("Lost Pack: ");    	Serial.println(lost_pack_number);
    Serial.println("------------------------------------------------------");
	#endif
}

void read_FM_UARTPort()
{
	recieve_pack();
    if (new_data == true)
    {
        strcpy(tempChars, receivedChars);
        parse_pack();
		//show_pack();
        new_data = false;
    }
}



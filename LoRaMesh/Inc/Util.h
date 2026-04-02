#ifndef INC_UTIL_H_
#define INC_UTIL_H_

extern uint32_t DEBUG_timestamp;

#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include "Packet_Handlers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

uint32_t Get_Timestamp();
void rand_delay();
uint32_t Get_Rand(uint32_t x);

#ifdef DEBUG
void DEBUG_Start_Timing();
uint32_t DEBUG_End_Timing();
#endif


#endif /* INC_UTIL_H_ */

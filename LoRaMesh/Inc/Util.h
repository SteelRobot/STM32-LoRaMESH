#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#ifdef DEBUG
#include <stdbool.h>
extern bool DEBUG_receive_to_send_flag;
extern uint32_t DEBUG_receive_to_send_timestamp;
extern uint32_t DEBUG_mesh_init_timestamp;
extern uint32_t DEBUG_lora_init_timestamp;
#endif

#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include "Packet_Handlers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

uint32_t Get_Timestamp(void);
void rand_delay(void);
uint32_t Get_Rand(uint32_t x);

#ifdef DEBUG
uint32_t DEBUG_Start_Timing(void);
uint32_t DEBUG_End_Timing(uint32_t timestamp);
#endif


#endif /* INC_UTIL_H_ */

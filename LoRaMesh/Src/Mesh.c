#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include "Packet_Handlers.h"
#include "Util.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define TX_SIZE 512

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

static uint32_t previous_timestamp = 0;

static uint8_t tx_buffer[TX_SIZE] = { 0 };
static uint8_t tx_byte;
static uint8_t tx_index = 0;

static uint32_t my_sequence_number;
static uint32_t rreq_id;
struct unicast_route_table_entry unicast_route_table[UNICAST_TABLE_LENGTH];
struct noroute_table_entry noroute_table[NOROUTE_QUEUE_MAX_ENTRIES];
uint8_t noroute_table_entries;
static uint32_t rreq_table[RREQ_TABLE_MAX_ENTRIES];
static uint8_t rreq_pointer;

// AODV
// No periodic beaconing
// Node floods the network with a Route REQuest (RREQ)
// Needed node replies with a Route REPly (RREP) on the first ping that it got

void Mesh_Init() {
	printf("Initializing Mesh Node\n");
#ifdef DEBUG
	DEBUG_Start_Timing();
#endif
	HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);

	if (my_id == 0) {
		printf("YOU MUST CHOOSE ANOTHER ADDRESS TO USE THIS NODE. ADDR = 0 IS RESERVED\n");
		exit(1);
	}

	Generate_RREQ_ID();
	my_sequence_number = 0;
	printf("Node ID: %d\n\n", my_id);
#ifdef DEBUG
	printf("Sequence Number: %ld, RREQ ID: %" PRIu32 "\n", my_sequence_number, rreq_id);
#endif
	Mesh_Send_Hello();
#ifdef DEBUG
	printf("\t\t\t\tDEBUG: Time to run Mesh_Init(): %ldms\n", DEBUG_End_Timing());
#endif
}

bool Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length) {
	bool isPing = false;

	if (my_id == destination_id) {
		printf("Sending a message to yourself? Abort\n");
		return SUCCESS;
	}

	if (data_length == 4 && memcmp(data, "ping", 4) == 0) {
#ifdef DEBUG
	printf("Handling a ping packet\n");
#endif
	    isPing = true;
	}
#ifdef DEBUG
	printf("Sending data over the Mesh\n");
#endif

	int8_t route_idx = Route_Exists(destination_id);
	Update_Routes_Expiration();
	if (route_idx != -1) {
#ifdef DEBUG
		printf("Route is known. Sending packet directly\n");
#endif
		struct unicast_route_table_entry route;
		route = unicast_route_table[route_idx];
		if (isPing) {
			Mesh_Send_PING(route.next_hop_destination_id, route.destination_id, my_id, 0, PING_REQUEST, Get_Timestamp());
		} else
			Mesh_Send_Data(route.destination_id, route.destination_sequence_number, data, route.next_hop_destination_id, 0, my_id, 0, data_length);
	} else {
#ifdef DEBUG
		printf("Route is not known. Sending a RREQ packet\n");
#endif

		Generate_RREQ_ID();
		Mesh_Send_RREQ(destination_id, my_id, my_sequence_number, 0, rreq_id);

#ifdef DEBUG
		printf("Adding the request to the no-route table\n");
#endif
		Noroute_Table_Add(destination_id, data, data_length);
	}
	return SUCCESS;
}

bool Noroute_Table_Add(uint16_t destination_id, uint8_t data[], uint8_t data_length) {

    if (data_length > MAX_PAYLOAD_SIZE) {
    	printf("Data was too large (%d byte) to be added to no-route table. Max length: %d\n", data_length, MAX_PAYLOAD_SIZE);
        return FAIL;
    }

	noroute_table[noroute_table_entries].destination_id = destination_id;
	memcpy(noroute_table[noroute_table_entries].data, data, data_length);
	noroute_table[noroute_table_entries].data_length = data_length;
	noroute_table_entries++;
	if (noroute_table_entries == NOROUTE_QUEUE_MAX_ENTRIES)
		noroute_table_entries = 0;
	return SUCCESS;
}

bool Mesh_Send_Hello() {
	printf("Broadcasting a hello message\n");
	uint8_t result = Mesh_Send_RREQ(0, my_id, Increment_Sequence_Number(), 0, rreq_id);
	return result;
}

int8_t Route_Exists(uint16_t id) {
#ifdef DEBUG
	printf("\tLooking for id = %d in unicast route table\n", id);
#endif
	for (int i = 0; i < UNICAST_TABLE_LENGTH; i++) {
		if (unicast_route_table[i].destination_id == id && unicast_route_table[i].expiration_time > 0) {
#ifdef DEBUG
			printf("\tFound entry for id=%d, next node is %d\n", unicast_route_table[i].destination_id, unicast_route_table[i].next_hop_destination_id);
#endif
			return i;
		}
	}
	return -1;
}

bool RREQ_Table_Contains(uint32_t rreq_id) {
	for (uint8_t i = 0; i < RREQ_TABLE_MAX_ENTRIES; i++) {
		if (rreq_table[i] == rreq_id) {
#ifdef DEBUG
			printf("RREQ Already Seen\n");
#endif
			return SUCCESS;
		}
	}
	return FAIL;
}

bool RREQ_Table_Append(uint32_t rreq_id) {
	rreq_table[rreq_pointer] = rreq_id;
	rreq_pointer++;
	if (rreq_pointer >= RREQ_TABLE_MAX_ENTRIES)
		rreq_pointer = 0;
	return SUCCESS;
}

bool Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num,
		uint8_t num_hops, uint16_t next_hop) {
#ifdef DEBUG
	printf("Checking to see if unicast table should be updated\n");
#endif

	int8_t source_idx = Route_Exists(dest_id);
	if (source_idx == -1) {
#ifdef DEBUG
		printf("\tSource node not found in table. Adding.\n");
#endif
		uint8_t replacement_route = Update_Routes_Expiration();
		unicast_route_table[replacement_route].destination_id = dest_id;
		unicast_route_table[replacement_route].destination_sequence_number = dest_seq_num;
		unicast_route_table[replacement_route].hop_count = num_hops;
		unicast_route_table[replacement_route].next_hop_destination_id = next_hop;
		unicast_route_table[replacement_route].expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
	} else {
		struct unicast_route_table_entry *existing_route = &unicast_route_table[source_idx];

		if (Is_Fresher_Route(dest_seq_num, existing_route->destination_sequence_number)) {
#ifdef DEBUG
			printf("\tNew route has fresher sequence number. Updating.\n");
#endif
			existing_route->destination_sequence_number = dest_seq_num;
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		} else if (dest_seq_num == existing_route->destination_sequence_number && num_hops < existing_route->hop_count) {
#ifdef DEBUG
			printf("\tSame sequence number, but new route is more efficient. Updating.\n");
#endif
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		} else {
#ifdef DEBUG
			printf("\tExisting route is fresher or equally good. Keeping.\n");
#endif
		}
	}

	return SUCCESS;
}

uint8_t Update_Routes_Expiration() { //Also returns id of the least fresh one
#ifdef DEBUG
	printf("Updating each route's freshness\n");
#endif
	uint32_t new_timestamp = Get_Timestamp() / 1000;
	uint32_t difference = 0;
	if (new_timestamp < previous_timestamp) //overflow control
		difference = new_timestamp;
	else
		difference = new_timestamp - previous_timestamp;
	previous_timestamp = new_timestamp;

	uint8_t route_idx = 0;
	uint8_t lowest_freshness = unicast_route_table[0].expiration_time;

	for (int i = 0; i < UNICAST_TABLE_LENGTH; i++) {
		if (unicast_route_table[i].expiration_time <= difference) { //overflow control
			unicast_route_table[i].expiration_time = 0;
			route_idx = i;
			lowest_freshness = 0;
		} else {
			unicast_route_table[i].expiration_time = unicast_route_table[i].expiration_time - previous_timestamp;
			if (unicast_route_table[i].expiration_time < lowest_freshness) {
				route_idx = i;
				lowest_freshness = unicast_route_table[i].expiration_time;
			}
		}
	}
#ifdef DEBUG
	printf("Returning route id: %d, time until expiration: %" PRIu32 "\n", route_idx, unicast_route_table[route_idx].expiration_time);
#endif

	return route_idx;
}

void Generate_RREQ_ID(void) {
	HAL_RTC_GetTime(Mesh_RTC, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(Mesh_RTC, &currentDate, RTC_FORMAT_BIN);

//	init_rng(currentTime.Seconds, currentTime.Minutes * 60, currentTime.SubSeconds);
	rreq_id = Get_Rand(currentTime.Seconds + currentTime.Minutes * 60 + currentTime.SubSeconds);
}

uint8_t Increment_Sequence_Number(void) {
	return ++my_sequence_number;
}

bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq) {
	return (int32_t) (new_seq - old_seq) > 0;
}

// UART Interrupt to receive data from LoRa module, and to send your own data from keyboard
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == COM_UART->Instance) {

//		uint32_t target_address = 0;
//		tx_buffer[tx_index] = tx_byte;
//		if (++tx_index >= TX_SIZE)
//			tx_index = 0;
//
//		if (tx_index > 2)
//			target_address = tx_buffer[0] - '0';
//
//		if (tx_byte == '\n' || tx_byte == '\r') {
//			memset(tx_data_to_send, 0, TX_SIZE);
//			memcpy(tx_data_to_send, tx_buffer, tx_index);
//			memset(tx_buffer, 0, TX_SIZE);
//			Mesh_Transmit(target_address, &tx_data_to_send[2], tx_index - 2);
//			tx_index = 0;
//		}

		tx_buffer[tx_index] = tx_byte;

		if (tx_byte == '\n' || tx_byte == '\r') {
			uint16_t dest_id = 0;
			uint8_t data_length = 0;
			char *space_pos = strchr((char*) tx_buffer, ' ');

			if (space_pos != NULL) {
				dest_id = atoi((char*) tx_buffer);

				uint8_t data_start = (space_pos - (char*) tx_buffer) + 1;
				data_length = tx_index - data_start;

				Mesh_Transmit(dest_id, &tx_buffer[data_start], data_length);
			}

			// Reset buffer and index
			memset(tx_buffer, 0, TX_SIZE);
			tx_index = 0;
		} else {
			// Increment index and wrap around if needed
			if (++tx_index >= TX_SIZE) {
				tx_index = 0;
			}
		}

		HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);
	}
}

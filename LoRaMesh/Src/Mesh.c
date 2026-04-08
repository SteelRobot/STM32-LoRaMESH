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

static uint8_t route_table_entries = 0;
static uint8_t rreq_table_entries = 0;

static uint8_t tx_buffer[TX_SIZE] = { 0 };
static uint8_t tx_byte;
static uint8_t tx_index = 0;

static uint32_t my_sequence_number;
static uint32_t rreq_id;
struct route_table_entry routing_table[ROUTING_TABLE_LENGTH];
struct pending_message pending_messages_table[PENDING_MESSAGES_TABLE_MAX_ENTRIES];
uint8_t pending_messages_table_entries;
static struct rreq_table_entry rreq_table[RREQ_TABLE_MAX_ENTRIES];

// AODV
// No periodic beaconing
// Node floods the network with a Route REQuest (RREQ)
// Needed node replies with a Route REPly (RREP) on the first ping that it got

void Mesh_Init() {
	printf("Initializing Mesh Node\n");
#ifdef DEBUG
	uint32_t DEBUG_mesh_init_timestamp = DEBUG_Start_Timing();
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
	printf("\t\t\t\tDEBUG: Time to run Mesh_Init(): %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_mesh_init_timestamp));
#endif
}

void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length) {
	bool isPing = false;

	printf("\n++++++++++++++++++++++++++++++++++++++++++");
	printf("\nYOU -> Node %d\n", destination_id);
	for (int i = 0; i < data_length; i++)
		printf("%c", data[i]);
	printf("\n++++++++++++++++++++++++++++++++++++++++++\n");

	if (destination_id == my_id) {
		printf("Sending a message to yourself? Abort\n");
		return;
	}
	else if (destination_id == 0xFFFF) {
		printf("You will NOT broadcast messages.\n");
		return;
	}
	else if (destination_id == 0x0000) {
		printf("This address is reserved for Hello messages.\n");
		return;
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
	if (route_idx != -1) {
#ifdef DEBUG
		printf("Route to %d is known. N of hops: %d\n", destination_id, routing_table[route_idx].hop_count);
#endif

		if (isPing) {
			Mesh_Send_Ping(routing_table[route_idx].next_hop_destination_id, routing_table[route_idx].destination_id, my_id, PING_REQUEST, Get_Timestamp());
		} else
			Mesh_Send_Data(routing_table[route_idx].destination_id, data, routing_table[route_idx].next_hop_destination_id, my_id, data_length);
	} else {
#ifdef DEBUG
		printf("Route to %d is NOT KNOWN. Sending a RREQ packet and adding the request to the no-route table\n", destination_id);
#endif

		Generate_RREQ_ID();
		Mesh_Send_RREQ(destination_id, my_id, Increment_Sequence_Number(), 0, rreq_id);
		Noroute_Table_Add(destination_id, data, data_length);
	}
}

void Noroute_Table_Add(uint16_t destination_id, uint8_t data[], uint8_t data_length) {

    if (data_length > MAX_PAYLOAD_SIZE) {
    	printf("Data was too large (%d byte) to be added to no-route table. Max length: %d\n", data_length, MAX_PAYLOAD_SIZE);
        return;
    }

	pending_messages_table[pending_messages_table_entries].destination_id = destination_id;
	memcpy(pending_messages_table[pending_messages_table_entries].data, data, data_length);
	pending_messages_table[pending_messages_table_entries].data_length = data_length;
	pending_messages_table_entries++;
	if (pending_messages_table_entries == PENDING_MESSAGES_TABLE_MAX_ENTRIES)
		pending_messages_table_entries = 0;
}

void Mesh_Send_Hello() {
#ifdef DEBUG
	printf("Broadcasting a hello message\n");
#endif
	Generate_RREQ_ID();
	Mesh_Send_RREQ(0, my_id, Increment_Sequence_Number(), 0, rreq_id);
}

int8_t Route_Exists(uint16_t id) {
#ifdef DEBUG
	printf("\tLooking for id = %d in unicast route table\n", id);
#endif
	Update_Routes_Expiration();
	for (int i = 0; i < ROUTING_TABLE_LENGTH; i++) {
		if (routing_table[i].destination_id == id && routing_table[i].expiration_time > 0) {
#ifdef DEBUG
			printf("\tFound entry for id=%d, next node is %d\n", routing_table[i].destination_id, routing_table[i].next_hop_destination_id);
#endif
			return i;
		}
	}
	return -1;
}

bool RREQ_Table_Contains(uint16_t source_id, uint32_t rreq_id) {
	Update_RREQ_Expiration();

	for (uint8_t i = 0; i < rreq_table_entries; i++) {
		if (rreq_table[i].source_id == source_id
				&& rreq_table[i].rreq_id == rreq_id) {
#ifdef DEBUG
			printf("RREQ Already Seen\n");
#endif
			return SUCCESS;
		}
	}
	return FAIL;
}

void RREQ_Table_Append(uint16_t source_id, uint32_t rreq_id) {
	if (rreq_table_entries >= RREQ_TABLE_MAX_ENTRIES)
		rreq_table_entries = 0;

	rreq_table[rreq_table_entries].source_id = source_id;
	rreq_table[rreq_table_entries].rreq_id = rreq_id;
	rreq_table[rreq_table_entries].expiration_time = Get_Timestamp() + (DEFAULT_RREQ_EXPIRATION_TIME * 1000);
	rreq_table_entries++;
}

void Update_RREQ_Expiration(void) {
#ifdef DEBUG
	printf("Updating each RREQ's freshness\n");
#endif
	uint32_t current_time = Get_Timestamp();
	uint8_t write_idx = 0;

	for (uint8_t read_idx = 0; read_idx < rreq_table_entries; read_idx++) {
		if (rreq_table[read_idx].expiration_time > current_time) {
#ifdef DEBUG
			printf("Current time: %" PRIu32 ", RREQ ID %" PRIu32 " from Node ID %d expires in %" PRIu32 " s\n",
					current_time / 1000, rreq_table[read_idx].rreq_id, rreq_table[read_idx].source_id, (rreq_table[read_idx].expiration_time - current_time) / 1000);
#endif
			rreq_table[write_idx] = rreq_table[read_idx];
			write_idx++;
		}
		//Skip
	}
	rreq_table_entries = write_idx;
}

void Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num,
		uint8_t num_hops, uint16_t next_hop) {
#ifdef DEBUG
	printf("Checking to see if unicast table should be updated\n");
#endif
	if (dest_id == 0x0000) // Reserved for Hello packets
		return;

	int8_t source_idx = Route_Exists(dest_id);
	if (source_idx == -1) {
#ifdef DEBUG
		printf("\tSource node not found in table. Adding.\n");
#endif

		if (route_table_entries >= ROUTING_TABLE_LENGTH)
			rreq_table_entries = 0;

		routing_table[route_table_entries].destination_id = dest_id;
		routing_table[route_table_entries].destination_sequence_number = dest_seq_num;
		routing_table[route_table_entries].hop_count = num_hops;
		routing_table[route_table_entries].next_hop_destination_id = next_hop;
		routing_table[route_table_entries].expiration_time = Get_Timestamp() + (DEFAULT_ROUTE_EXPIRATION_TIME * 1000);
		route_table_entries++;
	} else {
		struct route_table_entry *existing_route = &routing_table[source_idx];

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
}

void Update_Routes_Expiration(void) {
#ifdef DEBUG
	printf("Updating each route's freshness\n");
#endif
	uint32_t current_time = Get_Timestamp();
	uint8_t write_idx = 0;

	for (uint8_t read_idx = 0; read_idx < route_table_entries; read_idx++) {
		if (routing_table[read_idx].expiration_time > current_time) {
#ifdef DEBUG
			printf("Current time: %" PRIu32 ", route to %d expires in %" PRIu32 " s\n",
					current_time / 1000, routing_table[read_idx].destination_id, (routing_table[read_idx].expiration_time - current_time) / 1000);
#endif
			routing_table[write_idx] = routing_table[read_idx];
			write_idx++;
		}
		//Skip
	}

	route_table_entries = write_idx;
}

void Generate_RREQ_ID(void) {
	HAL_RTC_GetTime(Mesh_RTC, &currentTime, RTC_FORMAT_BIN);

	rreq_id = Get_Rand(currentTime.Seconds + currentTime.Minutes * 60 + currentTime.SubSeconds);
}

uint32_t Get_Sequence_Number(void) {
	return my_sequence_number;
}

uint32_t Increment_Sequence_Number(void) {
	return ++my_sequence_number;
}

bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq) {
	return (int32_t)(new_seq - old_seq) >= 0;
}

// UART Interrupt to receive data from LoRa module, and to send your own data from keyboard
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == COM_UART->Instance) {

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

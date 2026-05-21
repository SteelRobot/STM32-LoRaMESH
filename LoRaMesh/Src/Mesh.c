#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include "Packet_Handlers.h"
#include "Util.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define TX_SIZE 256

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

uint8_t route_table_entries = 0;
static uint8_t rreq_table_entries = 0;
static uint8_t tx_queue_head = 0;
static uint8_t tx_queue_tail = 0;

static uint8_t tx_buffer[TX_SIZE] = { 0 };
static uint8_t tx_byte;
static uint8_t tx_index = 0;

static uint32_t my_sequence_number = 0;
static uint32_t rreq_id;
route_table_entry routing_table[ROUTING_TABLE_LENGTH];
static rreq_table_entry rreq_table[RREQ_TABLE_MAX_ENTRIES];
tx_queue_entry tx_queue[TX_QUEUE_SIZE];

pending_message pending_buffer[MAX_PENDING_MESSAGES];

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
	printf("Node ID: %d\n\n", my_id);
	Mesh_Send_Hello();
#ifdef DEBUG
	printf("Sequence Number: %ld, RREQ ID: %" PRIu32 "\n", my_sequence_number, rreq_id);
#endif
#ifdef DEBUG
	printf("[T]\t\t\t\tDEBUG: Time to run Mesh_Init(): %" PRIu32 "ms\n", DEBUG_End_Timing(DEBUG_mesh_init_timestamp));
#endif
}

void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length) {

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

	bool is_ping = (data_length == 4 && memcmp(data, "ping", 4) == 0);
	int8_t route_idx = Route_Exists(destination_id);
	if (route_idx != -1) {
#ifdef DEBUG
		printf("Route to %d is known. N of hops: %d\n", destination_id, routing_table[route_idx].hop_count);
#endif
		printf("\n++++++++++++++++++++++++++++++++++++++++++");
		printf("\nYOU -> Node %d\n", destination_id);
		for (int i = 0; i < data_length; i++)
			printf("%c", data[i]);
		printf("\n++++++++++++++++++++++++++++++++++++++++++\n");
		if (is_ping)
			Mesh_Send_Ping(routing_table[route_idx].next_hop_destination_id,
					my_id,
					destination_id,
//					routing_table[route_idx].hop_count,
					Generate_Packet_ID(),
					MAX_HOPS_TTL,
					PING_REQUEST,
					Get_Timestamp(),
					true);
		else
			Mesh_Send_Data(routing_table[route_idx].next_hop_destination_id,
					my_id,
					destination_id,
//					routing_table[route_idx].hop_count,
					Generate_Packet_ID(),
					MAX_HOPS_TTL,
					data_length,
					data,
					true);
	} else {
#ifdef DEBUG
		printf("Route to %d is NOT KNOWN. Sending a RREQ packet and adding the request to the no-route table\n", destination_id);
#endif
		Buffer_Pending_Message(destination_id, data, data_length, is_ping);

		Mesh_Send_RREQ(my_id,
				destination_id,
				Increment_Sequence_Number(),
				0,
				Generate_RREQ_ID(),
				0,
				true);
	}
}

void Buffer_Pending_Message(uint16_t destination_id, uint8_t *data, uint8_t data_length, bool is_ping) {

    for (int i = 0; i < MAX_PENDING_MESSAGES; i++) {
    	pending_message *message = &pending_buffer[i];
        if (!message->in_use) {
        	message->destination_id = destination_id;
        	message->is_ping = is_ping;
        	message->timestamp_ms = Get_Timestamp();
        	message->in_use = true;
            if (!is_ping) {
            	message->data_length = data_length;
            	memcpy(message->data, data, data_length);
            }
#ifdef DEBUG
            printf("Buffered message for node %d\n", destination_id);
#endif
            return;
        }
    }
}

void Flush_Pending_Messages(uint16_t destination_id) {
    int8_t route_idx = Route_Exists(destination_id);
    if (route_idx == -1) return;

    for (int i = 0; i < MAX_PENDING_MESSAGES; i++) {
    	pending_message *message = &pending_buffer[i];
        if (message->in_use && message->destination_id == destination_id) {

            uint32_t age = Get_Timestamp() - message->timestamp_ms;
            if (age > PENDING_MSG_TIMEOUT_MS) {
#ifdef DEBUG
                printf("Discarding stale message for node %d\n", destination_id);
#endif
                message->in_use = true;
                continue;
            }

            if (message->is_ping)
                Mesh_Send_Ping(routing_table[route_idx].next_hop_destination_id,
                        my_id,
						destination_id,
//						routing_table[route_idx].hop_count,
						Generate_Packet_ID(),
                        MAX_HOPS_TTL,
						PING_REQUEST,
						Get_Timestamp(),
						true);
            else
                Mesh_Send_Data(routing_table[route_idx].next_hop_destination_id,
                        my_id,
						destination_id,
//						routing_table[route_idx].hop_count,
						Generate_Packet_ID(),
                        MAX_HOPS_TTL,
						pending_buffer[i].data_length,
                        pending_buffer[i].data,
						true);

            message->in_use = false;
        }
    }
}


bool TX_Queue_Push(uint8_t *serialized_data, uint16_t size,
                   uint8_t type, uint16_t destination_id,
//				   uint8_t hop_count,
				   bool is_originated,
				   uint8_t priority, uint32_t message_id, uint8_t max_retries) {
    uint8_t next_tail = (tx_queue_tail + 1) % TX_QUEUE_SIZE;
    if (next_tail == tx_queue_head) return FAIL;
    if (size > MAX_PAYLOAD_SIZE) return FAIL;

#ifdef DEBUG
    printf("Adding a new packet to the queue. Type %d\n", type);
#endif

    tx_queue_entry *entry = &tx_queue[tx_queue_tail];

    memcpy(entry->packet_data, serialized_data, size);
    entry->packet_size = size;
    entry->packet_type = type;
    entry->destination_id = destination_id;
//    entry->hop_count = hop_count;
    entry->is_originated = is_originated;
    entry->priority = priority;
    entry->message_id = message_id;
    entry->retry_count = 0;
    entry->max_retries = max_retries;
    entry->last_tx_time_ms = 0;
    entry->ack_received_hop_by_hop = false;
    entry->ack_received_end_to_end = false;
    entry->ready_to_send = true;

    tx_queue_tail = next_tail;
    return SUCCESS;
}

void TX_Queue_Pop(void) {
    while (tx_queue_head != tx_queue_tail) {
        tx_queue_entry *pkt = &tx_queue[tx_queue_head];

        bool is_sent_non_retryable = pkt->last_tx_time_ms != 0 &&
                                      (pkt->packet_type != DATA_PACKET &&
                                       pkt->packet_type != PING_PACKET);

        if (pkt->ack_received_end_to_end || is_sent_non_retryable) {
            tx_queue_head = (tx_queue_head + 1) % TX_QUEUE_SIZE;
        } else {
            break;
        }
    }
}

int8_t TX_Queue_Find_By_Message_ID(uint32_t message_id) {
	uint8_t i = tx_queue_head;
	while (i != tx_queue_tail) {
		if (tx_queue[i].message_id == message_id) {
			return i;
		}
		i = (i + 1) % TX_QUEUE_SIZE;
	}
	return -1;
}

void TX_Queue_Drop_By_Destination(uint16_t destination_id) {
    uint8_t i = tx_queue_head;

    while (i != tx_queue_tail) {
        tx_queue_entry *pkt = &tx_queue[i];

        if (pkt->destination_id == destination_id && !pkt->ack_received_end_to_end) {
            pkt->ack_received_end_to_end = true;

#ifdef DEBUG
            printf("Dropped packet to %d (type: %d, retries: %d)\n",
                   destination_id, pkt->packet_type, pkt->retry_count);
#endif
        }
        i = (i + 1) % TX_QUEUE_SIZE;
    }
}


tx_queue_entry* TX_Queue_Peek(void) {
    tx_queue_entry *highest = NULL;
    uint8_t check_count = 0;
    uint8_t i = tx_queue_head;

    while (i != tx_queue_tail && check_count < PRIORITY_PACKETS_AMOUNT) {
        tx_queue_entry *pkt = &tx_queue[i];

        if (pkt->ack_received_end_to_end) {
            i = (i + 1) % TX_QUEUE_SIZE;
            continue;
        }

        check_count++;

        if (!highest || pkt->priority > highest->priority) {
            highest = pkt;
        }

        i = (i + 1) % TX_QUEUE_SIZE;
    }
    return highest;
}




void TX_Queue_Handle_Packet_Failure(tx_queue_entry *pkt) {
    uint16_t failed_dest = pkt->destination_id;
    int8_t route_idx = Route_Exists(failed_dest);
    if (route_idx == -1) return;

    uint16_t dead_next_hop = routing_table[route_idx].next_hop_destination_id;
    uint16_t invalidated_dests[ROUTING_TABLE_LENGTH];
    uint8_t invalidated_count = 0;

    for (uint8_t j = 0; j < route_table_entries; j++) {
        if (routing_table[j].next_hop_destination_id == dead_next_hop &&
            routing_table[j].expiration_time > 0) {

            routing_table[j].expiration_time = 0;
            invalidated_dests[invalidated_count] = routing_table[j].destination_id;
            invalidated_count++;
        }
    }

    if (invalidated_count > 0) {
        for (uint8_t j = 0; j < invalidated_count; j++) {
            TX_Queue_Drop_By_Destination(invalidated_dests[j]);
        }
        Propagate_RERR_Upstream(invalidated_dests, invalidated_count);
    }
}

void TX_Queue_Check_Timeouts(void) {
    uint8_t i = tx_queue_head;
    uint32_t now = HAL_GetTick();

    while (i != tx_queue_tail) {
        tx_queue_entry *pkt = &tx_queue[i];

        if (!pkt->is_originated && pkt->ack_received_hop_by_hop) {
            pkt->ack_received_end_to_end = true;
            i = (i + 1) % TX_QUEUE_SIZE;
            continue;
        }

        if (pkt->ack_received_end_to_end) {
            i = (i + 1) % TX_QUEUE_SIZE;
            continue;
        }

        bool is_retryable = (pkt->packet_type == DATA_PACKET || pkt->packet_type == PING_PACKET);
        if (!is_retryable || pkt->last_tx_time_ms == 0) {
            i = (i + 1) % TX_QUEUE_SIZE;
            continue;
        }

        uint32_t elapsed = now - pkt->last_tx_time_ms;

        if (elapsed > PACKET_RETRY_TIME_MS) {
            if (pkt->retry_count < pkt->max_retries) {
                pkt->retry_count++;
                pkt->ready_to_send = true;
            } else {
                TX_Queue_Handle_Packet_Failure(pkt);
                pkt->ack_received_end_to_end = true;
            }
        }

        i = (i + 1) % TX_QUEUE_SIZE;
    }
}

void TX_Queue_Process(void) {
    TX_Queue_Check_Timeouts();

    tx_queue_entry *pkt = TX_Queue_Peek();
    if (pkt && pkt->ready_to_send) {
#ifdef DEBUG
        printf("Sending packet from queue. Type: %d\n", pkt->packet_type);
#endif
        LoRa_SendData(pkt->packet_data, pkt->packet_size);
        pkt->last_tx_time_ms = HAL_GetTick();
        pkt->ready_to_send = false;
    }

    TX_Queue_Pop();
}


void Mesh_Send_Hello(void) {
#ifdef DEBUG
	printf("Broadcasting a hello message\n");
#endif
	Mesh_Send_RREQ(my_id,
			0,
			Increment_Sequence_Number(),
			0,
			Generate_RREQ_ID(),
			0,
			true);
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
			printf("RREQ ID %" PRIu32 " from Node ID %d expires in %" PRIu32 " s\n",
					rreq_table[read_idx].rreq_id, rreq_table[read_idx].source_id, (rreq_table[read_idx].expiration_time - current_time) / 1000);
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

	int32_t new_expiration = Get_Timestamp() + (DEFAULT_ROUTE_EXPIRATION_TIME * 1000);
	int8_t source_idx = Route_Exists(dest_id);
	if (source_idx == -1) {
#ifdef DEBUG
		printf("\tSource node not found in table. Adding.\n");
#endif

		if (route_table_entries >= ROUTING_TABLE_LENGTH)
			route_table_entries = 0;

		routing_table[route_table_entries].destination_id = dest_id;
		routing_table[route_table_entries].destination_sequence_number = dest_seq_num;
		routing_table[route_table_entries].hop_count = num_hops;
		routing_table[route_table_entries].next_hop_destination_id = next_hop;
		routing_table[route_table_entries].expiration_time = new_expiration;
		routing_table[route_table_entries].precursor_count = 0;
		route_table_entries++;
	} else {
		route_table_entry *existing_route = &routing_table[source_idx];

		if (Is_Fresher_Route(dest_seq_num, existing_route->destination_sequence_number)) {
#ifdef DEBUG
			printf("\tNew route has fresher sequence number. Updating.\n");
#endif
			existing_route->destination_sequence_number = dest_seq_num;
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = new_expiration;
		} else if (dest_seq_num == existing_route->destination_sequence_number && num_hops < existing_route->hop_count) {
#ifdef DEBUG
			printf("\tSame sequence number, but new route is more efficient. Updating.\n");
#endif
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = new_expiration;
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
			printf("Route to %d expires in %" PRIu32 " s\n",
					routing_table[read_idx].destination_id, (routing_table[read_idx].expiration_time - current_time) / 1000);
#endif
			routing_table[write_idx] = routing_table[read_idx];
			write_idx++;
		}
		//Skip
	}

	route_table_entries = write_idx;
}

void Add_To_Precursor_List(uint8_t route_idx, uint16_t precursor_id) {
	if (precursor_id == 0x0000 || precursor_id == 0xFFFF)
		return;

	route_table_entry *route = &routing_table[route_idx];

	for (uint8_t i = 0; i < route->precursor_count; i++) {
		if (route->precursor_list[i] == precursor_id)
			return;
	}

	if (route->precursor_count < MAX_PRECURSORS) {
		route->precursor_list[route->precursor_count] = precursor_id;
		route->precursor_count++;
#ifdef DEBUG
		printf("Added precursor %d to route %d\n",
				precursor_id, route->destination_id);
#endif
	}
}

uint32_t Generate_RREQ_ID(void) {
	HAL_RTC_GetTime(Mesh_RTC, &currentTime, RTC_FORMAT_BIN);

	rreq_id = Get_Rand(currentTime.Seconds + currentTime.Minutes * 60 + currentTime.SubSeconds);
	return rreq_id;
}

uint32_t Generate_Packet_ID(void) {
	HAL_RTC_GetTime(Mesh_RTC, &currentTime, RTC_FORMAT_BIN);

	return Get_Rand(currentTime.Seconds + currentTime.Minutes * 60 + currentTime.SubSeconds);
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

			memset(tx_buffer, 0, TX_SIZE);
			tx_index = 0;
		} else {
			if (++tx_index >= TX_SIZE) {
				tx_index = 0;
			}
		}

		HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);
	}
}

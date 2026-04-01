#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define TX_SIZE 512

static uint8_t tx_data_to_send[TX_SIZE] = { 0 };
static uint8_t tx_buffer[TX_SIZE] = { 0 };
static uint8_t tx_byte;
static uint8_t tx_index = 0;

static uint16_t my_id;
static uint8_t my_channel;
static int offset = 0;
static uint32_t my_sequence_number;
static uint32_t rreq_id;
static uint8_t rx_data[256];
static uint32_t unicast_entries;
static struct unicast_route_table_entry unicast_route_table[UNICAST_TABLE_LENGTH];
static struct noroute_table_entry noroute_table[NOROUTE_QUEUE_MAX_ENTRIES];
static uint8_t noroute_table_entries;
static uint32_t rreq_table[RREQ_TABLE_MAX_ENTRIES];
static uint8_t rreq_pointer;

// AODV
// No periodic beaconing
// Node floods the network with a Route REQuest (RREQ)
// Needed node replies with a Route REPly (RREP) on the first ping that it got

void Mesh_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2,
		DMA_HandleTypeDef *hdma_usart1_rx) {
	printf("Initializing Mesh Node\n");
	LoRa_Init(huart1, huart2, hdma_usart1_rx);

	HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);

	my_id = current_node_address;

	if (my_id == 0) {
		printf("YOU MUST CHOOSE ANOTHER ADDRESS TO USE THIS NODE. ADDR = 0 IS RESERVED\n");
		exit(1);
	}

	my_channel = current_node_channel;
	srand(my_id);
	rreq_id = rand();
	my_sequence_number = 0;
	Update_Route_Table(my_id, my_sequence_number, 0, my_id);
	printf("Node ID: %d, Sequence Number: %ld, RREQ ID: %ld\n", my_id,
			my_sequence_number, rreq_id);
	Mesh_Send_Hello();
}

uint8_t Mesh_Transmit(uint16_t destination_id, uint8_t data[],
		uint8_t data_length) {

	if (my_id == destination_id) {
		printf("Sending a message to yourself? Abort\n");
		return SUCCESS;
	}

	printf("Sending data over the Mesh\n");

	int8_t route_idx = Route_Exists(destination_id);
	if (route_idx != -1) {
		printf("Route is known. Sending data directly\n");
		struct unicast_route_table_entry route;
		route = unicast_route_table[route_idx];
		Mesh_Send_Data(route.destination_id, route.destination_sequence_number,
				data, route.next_hop_destination_id, 0, my_id, 0, data_length);
	} else {
		printf("Route is not known. Sending a RREQ packet\n");

		rreq_id++;
		Mesh_Send_RREQ(destination_id, my_id, my_sequence_number, 0, rreq_id);

		printf("Adding the request to the no-route table\n");
		noroute_table[noroute_table_entries].destination_id = destination_id;
		noroute_table[noroute_table_entries].data = data;
		noroute_table[noroute_table_entries].data_length = data_length;
		noroute_table_entries++;
	}
	return SUCCESS;
}

uint8_t Mesh_Send_Data(uint16_t destination_id, uint32_t dest_seq_num,
		uint8_t *data, uint16_t receiver_id, uint8_t num_hops,
		uint16_t source_id, uint32_t source_sequence_number, uint8_t data_length) {
	printf("Building a Data Packet\n");

	struct data_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.packet_data = data;
	tosend.data_length = data_length;
	tosend.destination_sequence_number = dest_seq_num;
	tosend.source_sequence_number = source_sequence_number;

	printf("Transmitting a Data Packet to Node %d\n", destination_id);
	uint8_t packet_arr[DATA_PREAMBLE_PKT_LEN + data_length];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_Data(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, DATA_PREAMBLE_PKT_LEN + data_length);
	return result;
}

uint8_t Mesh_Send_RREQ(uint16_t destination_id, uint16_t source_id,
		uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id) {
	printf("Building an RREQ Packet\n");
	printf("RREQ ID %ld\n", rreq_id);
	if (source_id == my_id) {
		source_sequence_number = Increment_Sequence_Number();
	}

	struct rreq_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.source_sequence_number = source_sequence_number;
	tosend.rreq_id = rreq_id;

	RREQ_Table_Append(rreq_id);

	printf("Broadcasting the RREQ\n");

	uint8_t packet_arr[RREQ_PREAMBLE_PKT_LEN];

	packet_arr[0] = 0xFF;
	packet_arr[1] = 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_RREQ(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, RREQ_PREAMBLE_PKT_LEN);
	return result;
}

uint8_t Mesh_Send_RREP(uint16_t receiver_id, uint16_t destination_id,
		uint16_t source_id, uint8_t num_hops, uint32_t dest_seq_num) {
	printf("Building an RREP Packet\n");

	struct rrep_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.destination_sequence_number = dest_seq_num;

	uint8_t packet_arr[RREP_PREAMBLE_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_RREP(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, RREP_PREAMBLE_PKT_LEN);
	return result;
}

uint8_t Mesh_Send_Hello() {
	printf("Broadcasting a hello message\n");
	uint8_t result = Mesh_Send_RREQ(0, my_id, Increment_Sequence_Number(), 0, rreq_id);
	return result;
}

static void Write_uint8(uint8_t *arr, uint8_t value) {
	arr[offset++] = value & 0xFF;
}

static void Write_uint16(uint8_t *arr, uint16_t value) {
	arr[offset++] = (value >> 8) & 0xFF;
	arr[offset++] = value & 0xFF;
}

static void Write_uint32(uint8_t *arr, uint32_t value) {
	arr[offset++] = (value >> 24) & 0xFF;
	arr[offset++] = (value >> 16) & 0xFF;
	arr[offset++] = (value >> 8) & 0xFF;
	arr[offset++] = value & 0xFF;
}

static uint8_t Read_uint8(const uint8_t *arr) {
	return arr[offset++];
}

static uint16_t Read_uint16(const uint8_t *arr) {
	uint16_t value =
			(arr[offset + 0] << 8) |
			arr[offset + 1];
	offset += 2;
	return value;

}
static uint32_t Read_uint32(const uint8_t *arr) {
	uint32_t value =
			(arr[offset + 0] << 24) 	|
			(arr[offset + 1] << 16) 	|
			(arr[offset + 2] << 8) 	|
			arr[offset + 3];
	offset += 4;
	return value;
}

uint8_t Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, DATA_PACKET);
	Write_uint8(packet_arr, 0);
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint32(packet_arr, packet.source_sequence_number);
	memcpy(&packet_arr[offset], packet.packet_data, packet.data_length);

	offset = 0;
	return SUCCESS;
}

uint8_t Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREQ_PACKET);
	Write_uint8(packet_arr, 0);
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint32(packet_arr, packet.rreq_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint32(packet_arr, packet.source_sequence_number);

	offset = 0;
	return SUCCESS;
}

uint8_t Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREP_PACKET);
	Write_uint8(packet_arr, 0);
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint16(packet_arr, packet.source_id);

	offset = 0;
	return SUCCESS;
}

struct data_packet Unpack_Packet_Data(uint8_t parr[], uint8_t data_length,
		uint8_t data[]) {
	offset = 0;

	struct data_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t reserved_1 =
	Read_uint8(parr);
	//	uint8_t reserved_2 =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.receiver_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.destination_sequence_number = Read_uint32(parr);
	packet.source_sequence_number = Read_uint32(parr);

	packet.data_length = data_length;
	if (data_length > 0 && data != NULL) {
		memcpy(data, &parr[offset], data_length);
		packet.packet_data = data;
	}

	offset = 0;

	printf("\tData packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id);
	return packet;
}

struct rreq_packet Unpack_Packet_RREQ(uint8_t parr[]) {
	offset = 0;

	struct rreq_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t reserved_1 =
	Read_uint8(parr);
	//	uint8_t reserved_2 =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.rreq_id = Read_uint32(parr);
	packet.destination_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.source_sequence_number = Read_uint32(parr);

	offset = 0;

	printf("\tRREQ packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\trreq_id=%ld\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.rreq_id,
			packet.destination_id, packet.source_id);

	return packet;
}

//fills an rrep packet struct from an array
struct rrep_packet Unpack_Packet_RREP(uint8_t parr[]) {
	offset = 0;

	struct rrep_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t reserved_1 =
	Read_uint8(parr);
//	uint8_t reserved_2 =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.receiver_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.destination_sequence_number = Read_uint32(parr);
	packet.source_id = Read_uint16(parr);

	offset = 0;

	printf("\tRREP packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id);

	return packet;
}

uint8_t Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength) {
	printf("Handling a new packet\n");
	uint8_t ptype = packet_data[0] & 0xFF;
	printf("\tPacket type is %d\n", ptype);

	switch (ptype) {
	case RREQ_PACKET:
		return Receive_Packet_Handler_RREQ(packet_data, plength);
	case RREP_PACKET:
		return Receive_Packet_Handler_RREP(packet_data, plength);
	case DATA_PACKET:
		return Receive_Packet_Handler_Data(packet_data, plength);
	case INVALID_PACKET:
		return Receive_Packet_Handler_Invalid(packet_data, plength);
	default:
		return Receive_Packet_Handler_Invalid(packet_data, plength);
	}
	return SUCCESS;
}

uint8_t Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength) {
	struct rreq_packet pkt;
	pkt = Unpack_Packet_RREQ(packet_data);

	if (!RREQ_Table_Contains(pkt.rreq_id)) {
		printf("Valid RREQ\n");
		RREQ_Table_Append(pkt.rreq_id);
		Update_Route_Table(pkt.source_id, pkt.source_sequence_number, pkt.num_hops, pkt.transmitter_id);
		Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
		if (pkt.destination_id == my_id) {
			uint32_t my_dest_seq = Increment_Sequence_Number();
			Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id, pkt.destination_id, 0, my_dest_seq);
		} else {
			int8_t route_idx = Route_Exists(pkt.destination_id);
			if (route_idx != -1) {
				uint32_t known_dest_seq = unicast_route_table[route_idx].destination_sequence_number;
				Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id,
						pkt.destination_id,
						unicast_route_table[route_idx].hop_count,
						known_dest_seq);
			} else {
				Mesh_Send_RREQ(pkt.destination_id, pkt.source_id,
						pkt.source_sequence_number, pkt.num_hops + 1,
						pkt.rreq_id);
			}
		}
		if (pkt.destination_id == 0) {
			printf("This is a hello packet from node %d\n", pkt.source_id);
		}
	} else {
		printf("Stale RREQ %ld\n", pkt.rreq_id);
	}

	return SUCCESS;
}

uint8_t Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength) {
	struct rrep_packet pkt;
	pkt = Unpack_Packet_RREP(packet_data);

	if (pkt.receiver_id == my_id) {
		Update_Route_Table(pkt.source_id, pkt.destination_sequence_number,
				pkt.num_hops, pkt.transmitter_id);
		Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
	}

	if (pkt.destination_id == my_id && pkt.receiver_id == my_id) {
		printf("RREP Packet for me\n");

		unicast_route_table[unicast_entries].destination_id = pkt.source_id;
		unicast_route_table[unicast_entries].hop_count = pkt.num_hops;
		unicast_route_table[unicast_entries].next_hop_destination_id =
				pkt.transmitter_id;
		unicast_route_table[unicast_entries].expiration_time =
		DEFAULT_ROUTE_EXPIRATION_TIME;
		unicast_entries++;

		printf(
				"Searching for noroute table for blocked requests, with %d entries\n",
				noroute_table_entries);
		for (int i = 0; i < noroute_table_entries; i++) {
			if (noroute_table[i].destination_id == pkt.source_id) {
				printf("Found a matching entry in the noroute table\n");
				Mesh_Transmit(noroute_table[i].destination_id,
						noroute_table[i].data, noroute_table[i].data_length);
			}
		}
	} else if (pkt.receiver_id == my_id) {
		int8_t route_idx = Route_Exists(pkt.destination_id);
		Mesh_Send_RREP(unicast_route_table[route_idx].next_hop_destination_id,
				pkt.destination_id, pkt.source_id, pkt.num_hops + 1,
				pkt.destination_sequence_number);
	}
	return SUCCESS;
}

uint8_t Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength) {
	struct data_packet pkt;
	pkt = Unpack_Packet_Data(packet_data, (plength - DATA_BASE_PKT_LEN),
			rx_data);

	if (pkt.receiver_id == my_id) {
		Update_Route_Table(pkt.source_id, pkt.source_sequence_number,
				pkt.num_hops, pkt.transmitter_id);
		Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
	}

	if (pkt.destination_id == my_id && pkt.receiver_id == my_id) {
		DATA_RX_HANDLER(pkt);
	} else if (pkt.receiver_id == my_id) {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1) {
			struct unicast_route_table_entry route;
			route = unicast_route_table[route_idx];
			Mesh_Send_Data(route.destination_id,
					route.destination_sequence_number, pkt.packet_data,
					route.next_hop_destination_id, pkt.num_hops + 1,
					pkt.source_id, pkt.source_sequence_number, pkt.data_length);
		}
	}
	return SUCCESS;
}

uint8_t Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength) {
	printf("Invalid Packet Type (data[0] = %d\n", packet_data[0]);
	return FAIL;
}

int8_t Route_Exists(uint16_t id) {
	printf("\tLooking for id = %d in unicast route table\n", id);
	for (int i = 0; i < unicast_entries; i++) {
		if (unicast_route_table[i].destination_id == id) {
			printf("\tFound entry for id=%d, next node is %d\n",
					unicast_route_table[i].destination_id,
					unicast_route_table[i].next_hop_destination_id);
			return i;
		}
	}
	return -1;
}

uint8_t RREQ_Table_Contains(uint32_t rreq_id) {
	for (uint8_t i = 0; i < RREQ_TABLE_MAX_ENTRIES; i++) {
		if (rreq_table[i] == rreq_id) {
			printf("RREQ Already Seen\n");
			return 1;
		}
	}
	return 0;
}

uint8_t RREQ_Table_Append(uint32_t rreq_id) {
	rreq_table[rreq_pointer] = rreq_id;
	rreq_pointer++;
	if (rreq_pointer >= RREQ_TABLE_MAX_ENTRIES)
		rreq_pointer = 0;
	return SUCCESS;
}

uint8_t Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num,
		uint8_t num_hops, uint16_t next_hop) {
	printf("Checking to see if unicast table should be updated\n");

	int8_t source_idx = Route_Exists(dest_id);
	if (source_idx == -1) {
		printf("\tSource node not found in table. Adding.\n");
		unicast_route_table[unicast_entries].destination_id = dest_id;
		unicast_route_table[unicast_entries].destination_sequence_number = dest_seq_num;
		unicast_route_table[unicast_entries].hop_count = num_hops;
		unicast_route_table[unicast_entries].next_hop_destination_id = next_hop;
		unicast_route_table[unicast_entries].expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		unicast_entries++;
	} else {
		struct unicast_route_table_entry *existing_route = &unicast_route_table[source_idx];

		if (Is_Fresher_Route(dest_seq_num, existing_route->destination_sequence_number)) {
			printf("\tNew route has fresher sequence number. Updating.\n");
			existing_route->destination_sequence_number = dest_seq_num;
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		} else if (dest_seq_num == existing_route->destination_sequence_number && num_hops < existing_route->hop_count) {
			printf("\tSame sequence number, but new route is more efficient. Updating.\n");
			existing_route->hop_count = num_hops;
			existing_route->next_hop_destination_id = next_hop;
			existing_route->expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		} else {
			printf("\tExisting route is fresher or equally good. Keeping.\n");
		}
	}

	return SUCCESS;
}

void rand_delay() {
	HAL_Delay(1);
}

uint8_t Increment_Sequence_Number(void) {
	return ++my_sequence_number;
}

uint8_t Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq) {
	return (int32_t) (new_seq - old_seq) > 0;
}

uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt) {
	//Print the received data to the console
	printf("Received Data: ");
	printf((char*) rx_pkt.packet_data);
	printf("\n");
	return SUCCESS;
}

// UART Interrupt to receive data from LoRa module, and to send your own data from keyboard
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == COM_UART->Instance) {

		uint32_t target_address;
		tx_buffer[tx_index] = tx_byte;
		if (++tx_index >= TX_SIZE)
			tx_index = 0;

		if (tx_index > 2)
			target_address = tx_buffer[0] - '0';
		//TODO() Write a small Python script to be able to input address as hex and not string

		if (tx_byte == '\n' || tx_byte == '\r') {
			memset(tx_data_to_send, 0, TX_SIZE);
			memcpy(tx_data_to_send, tx_buffer, tx_index);
			memset(tx_buffer, 0, TX_SIZE);
			Mesh_Transmit(target_address, &tx_data_to_send[2], tx_index - 2);
			tx_index = 0;
		}
		HAL_UART_Receive_IT(COM_UART, &tx_byte, 1);
	}
}

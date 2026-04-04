#include "main.h"
#include "Packet_Handlers.h"
#include "Mesh.h"
#include "LoRa.h"
#include "Util.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static int offset = 0;
static uint8_t rx_data[256];
static uint8_t payload_length = 0;

bool Mesh_Send_Data(uint16_t destination_id, uint32_t dest_seq_num,
		uint8_t *data, uint16_t receiver_id, uint8_t num_hops,
		uint16_t source_id, uint32_t source_sequence_number, uint8_t data_length) {
#ifdef DEBUG
	printf("Building a Data Packet\n");
#endif

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

#ifdef DEBUG
	printf("\tdata_length=%d\n\tnum_hops=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			data_length, num_hops, receiver_id,
			destination_id, source_id);
#endif
	uint8_t packet_arr[DATA_PKT_LEN + data_length];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_Data(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, DATA_PKT_LEN + data_length);
	return result;
}

bool Mesh_Send_RREQ(uint16_t destination_id, uint16_t source_id,
		uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id) {
#ifdef DEBUG
	printf("Building an RREQ Packet\n");
	printf("%" PRIu32 "\n", rreq_id);
#endif

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
#ifdef DEBUG
	printf("\tnum_hops=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			num_hops, 0xFFFF,
			destination_id, source_id);
#endif
	RREQ_Table_Append(rreq_id);

	uint8_t packet_arr[RREQ_PKT_LEN];

	packet_arr[0] = 0xFF;
	packet_arr[1] = 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_RREQ(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, RREQ_PKT_LEN);
	return result;
}

bool Mesh_Send_RREP(uint16_t receiver_id, uint16_t destination_id,
		uint16_t source_id, uint8_t num_hops, uint32_t dest_seq_num) {
#ifdef DEBUG
	printf("Building an RREP Packet\n");
#endif

	struct rrep_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.destination_sequence_number = dest_seq_num;
#ifdef DEBUG
	printf("\tnum_hops=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			num_hops, receiver_id,
			destination_id, source_id);
#endif
	uint8_t packet_arr[RREP_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_RREP(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, RREP_PKT_LEN);
	return result;
}

bool Mesh_Send_PING(uint16_t receiver_id, uint16_t destination_id, uint16_t source_id, uint8_t num_hops, uint8_t request_or_reply, uint32_t timestamp_ms) {
#ifdef DEBUG
	printf("Building a PING Packet\n");
#endif

	struct ping_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.request_or_reply = request_or_reply;
	tosend.timestamp_ms = timestamp_ms;
#ifdef DEBUG
	printf("\tnum_hops=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			num_hops, receiver_id,
			destination_id, source_id);
#endif
	uint8_t packet_arr[PING_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	uint8_t result = Format_Packet_PING(tosend, packet_arr + LORA_OFFSET);
	result &= LoRa_SendData(packet_arr, PING_PKT_LEN);
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

static uint8_t Start_Length_Count() {
	payload_length = offset + 1;
	return 0;
}

static void End_Length_Count(uint8_t packet_arr[]) {
	payload_length = offset - payload_length;
	packet_arr[LENGTH_BYTE_POS] = payload_length;
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
			(arr[offset + 2] << 8) 		|
			arr[offset + 3];
	offset += 4;
	return value;
}

bool Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, DATA_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint32(packet_arr, packet.source_sequence_number);
	memcpy(&packet_arr[offset], packet.packet_data, packet.data_length);
	offset += packet.data_length;

	End_Length_Count(packet_arr);

	offset = 0;
	return SUCCESS;
}

bool Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREQ_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint32(packet_arr, packet.rreq_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint32(packet_arr, packet.source_sequence_number);

	End_Length_Count(packet_arr);

	offset = 0;
	return SUCCESS;
}

bool Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREP_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint16(packet_arr, packet.source_id);

	End_Length_Count(packet_arr);

	offset = 0;
	return SUCCESS;
}

bool Format_Packet_PING(struct ping_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, PING_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint8(packet_arr, packet.request_or_reply);
	Write_uint32(packet_arr, packet.timestamp_ms);

	End_Length_Count(packet_arr);

	offset = 0;
	return SUCCESS;
}

struct data_packet Unpack_Packet_Data(uint8_t parr[], uint8_t data_length,
		uint8_t data[]) {
	offset = 0;

	struct data_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t payload_length =
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

#ifdef DEBUG
	printf("\tData packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id);
#endif
	return packet;
}

struct rreq_packet Unpack_Packet_RREQ(uint8_t parr[]) {
	offset = 0;

	struct rreq_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t payload_length =
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

#ifdef DEBUG
	printf("\tRREQ packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\trreq_id=%" PRIu32 "\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.rreq_id,
			packet.destination_id, packet.source_id);
#endif
	return packet;
}

//fills an rrep packet struct from an array
struct rrep_packet Unpack_Packet_RREP(uint8_t parr[]) {
	offset = 0;

	struct rrep_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
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

#ifdef DEBUG
	printf("\tRREP packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id);
#endif
	return packet;
}

struct ping_packet Unpack_Packet_PING(uint8_t parr[]) {
	offset = 0;

	struct ping_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved_2 =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.receiver_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.request_or_reply = Read_uint8(parr);
	packet.timestamp_ms = Read_uint32(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tPING packet with:\n");
	printf(
			"\tnum_hops = %d\n\ttransmitter_id=%d\n\treceiver_id=%d\n\tdestination_id=%d\n\tsource_id=%d\nRequest or Reply:%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id, packet.request_or_reply);
#endif
	return packet;
}

bool Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength, uint8_t ptype) {
#ifdef DEBUG
	printf("Handling a new packet\n");
	switch (ptype) {
	case RREQ_PACKET:
		printf("\tPacket type is RREQ\n");
		break;
	case RREP_PACKET:
		printf("\tPacket type is RREP\n");
		break;
	case DATA_PACKET:
		printf("\tPacket type is DATA\n");
		break;
	case PING_PACKET:
		printf("\tPacket type is PING\n");
		break;
	case INVALID_PACKET:
		printf("\tPacket type is INVALID\n");
		break;
	default:
		printf("\tPacket type is INVALID\n");
		break;
	}
#endif

	switch (ptype) {
	case RREQ_PACKET:
		return Receive_Packet_Handler_RREQ(packet_data, plength);
	case RREP_PACKET:
		return Receive_Packet_Handler_RREP(packet_data, plength);
	case DATA_PACKET:
		return Receive_Packet_Handler_Data(packet_data, plength);
	case PING_PACKET:
		return Receive_Packet_Handler_Ping(packet_data, plength);
	case INVALID_PACKET:
		return Receive_Packet_Handler_Invalid(packet_data, plength);
	default:
		return Receive_Packet_Handler_Invalid(packet_data, plength);
	}
	return SUCCESS;
}

bool Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength) {
	struct rreq_packet pkt;
	pkt = Unpack_Packet_RREQ(packet_data);

	if (RREQ_Table_Contains(pkt.rreq_id)) {
#ifdef DEBUG
		printf("Stale RREQ %" PRIu32 "\n", pkt.rreq_id);
#endif
		return SUCCESS;
	}

#ifdef DEBUG
	printf("Valid RREQ\n");
#endif
	RREQ_Table_Append(pkt.rreq_id);
	Update_Route_Table(pkt.source_id, pkt.source_sequence_number, pkt.num_hops,
			pkt.transmitter_id);
	Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
	if (pkt.destination_id == my_id) {
		uint32_t my_dest_seq = Increment_Sequence_Number();
		Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id, pkt.destination_id, 0,
				my_dest_seq);
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);
		if (route_idx != -1) {
			uint32_t known_dest_seq =
					unicast_route_table[route_idx].destination_sequence_number;
			Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id,
					pkt.destination_id,
					unicast_route_table[route_idx].hop_count, known_dest_seq);
		} else {
			Mesh_Send_RREQ(pkt.destination_id, pkt.source_id,
					pkt.source_sequence_number, pkt.num_hops + 1, pkt.rreq_id);
		}
	}
	if (pkt.destination_id == 0) {
		printf("Received a Hello packet from node %d\n", pkt.source_id);
	}

	return SUCCESS;
}

bool Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength) {
	struct rrep_packet pkt;
	pkt = Unpack_Packet_RREP(packet_data);

	Update_Route_Table(pkt.source_id, pkt.destination_sequence_number, pkt.num_hops, pkt.transmitter_id);
	Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);

	if (pkt.destination_id == my_id) {
		printf("RREP Packet for me\n");

#ifdef DEBUG
		printf("Searching noroute table for blocked requests, with %d entries\n", noroute_table_entries);
#endif
		for (int i = 0; i < noroute_table_entries; i++) {
			if (noroute_table[i].destination_id == pkt.source_id) {
#ifdef DEBUG
				printf("Found a matching entry in the noroute table\n");
#endif
				Mesh_Transmit(noroute_table[i].destination_id,
						noroute_table[i].data, noroute_table[i].data_length);
			}
		}
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);
		Mesh_Send_RREP(unicast_route_table[route_idx].next_hop_destination_id,
				pkt.destination_id, pkt.source_id, pkt.num_hops + 1,
				pkt.destination_sequence_number);
	}
	return SUCCESS;
}

bool Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength) {
	struct data_packet pkt;
	pkt = Unpack_Packet_Data(packet_data, (plength - DATA_BASE_PKT_LEN), rx_data);

	Update_Route_Table(pkt.source_id, pkt.source_sequence_number, pkt.num_hops, pkt.transmitter_id);
	Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);

	if (pkt.destination_id == my_id) {
		DATA_RX_HANDLER(pkt);
	} else {
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

bool Receive_Packet_Handler_Ping(uint8_t packet_data[], uint8_t plength) {
	struct ping_packet pkt;
	pkt = Unpack_Packet_PING(packet_data);

	if (pkt.destination_id == my_id) {
		if (pkt.request_or_reply == PING_REQUEST) {
			Mesh_Send_PING(pkt.transmitter_id, pkt.source_id, pkt.destination_id, 0, PING_REPLY, pkt.timestamp_ms);
		} else {
			printf("Received ping reply from node id:%d\n", pkt.source_id);
			printf("Latency: %" PRIu32 "ms\n", Get_Timestamp() - pkt.timestamp_ms);
			return SUCCESS;
		}
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1) {
			struct unicast_route_table_entry route;
			route = unicast_route_table[route_idx];
			Mesh_Send_PING(route.next_hop_destination_id, route.destination_id, pkt.source_id, pkt.num_hops + 1, pkt.request_or_reply, pkt.timestamp_ms);
		}
	}
	return SUCCESS;
}


bool Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength) {
#ifdef DEBUG
	printf("Invalid Packet Type (data[0] = %d)\n", packet_data[0]);
#endif
	return false;
}

bool DATA_RX_HANDLER(struct data_packet rx_pkt) {
	//Print the received data to the console
	printf("##########################################\n");
	printf("Node %d:\n", rx_pkt.source_id);
	for (int i = 0; i < rx_pkt.data_length; i++)
		printf("%c", rx_pkt.packet_data[i]);
	printf("\n##########################################\n");
	return SUCCESS;
}

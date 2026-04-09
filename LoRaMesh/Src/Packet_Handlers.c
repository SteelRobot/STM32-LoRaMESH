#include "main.h"
#include "Packet_Handlers.h"
#include "Mesh.h"
#include "LoRa.h"
#include "Util.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static uint8_t offset = 0;
static uint8_t rx_data[256];
static uint8_t payload_length = 0;

void Mesh_Send_Data(uint16_t destination_id,
		uint8_t *data, uint16_t receiver_id,
		uint16_t source_id, uint8_t data_length) {
#ifdef DEBUG
	printf("Building a Data Packet\n");
#endif

	offset = 0;

	struct data_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.packet_data = data;
	tosend.data_length = data_length;

#ifdef DEBUG
	printf("\tdata_length=%d receiver_id=%d destination_id=%d source_id=%d\n",
			data_length, receiver_id,
			destination_id, source_id);
#endif
	uint8_t packet_arr[DATA_PKT_LEN + data_length];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_Data(tosend, packet_arr + LORA_OFFSET);
	LoRa_SendData(packet_arr, DATA_PKT_LEN + data_length);
	offset = 0;
}

void Mesh_Send_RREQ(uint16_t destination_id, uint32_t destination_sequence_number, uint16_t source_id,
		uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id) {

#ifdef DEBUG
	printf("Building an RREQ Packet\n");
	printf("%" PRIu32 "\n", rreq_id);
#endif

	offset = 0;

	struct rreq_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.source_sequence_number = source_sequence_number;
	tosend.destination_sequence_number = destination_sequence_number;
	tosend.rreq_id = rreq_id;

#ifdef DEBUG
	printf("\tnum_hops=%d transmitter_id=%d destination_id=%d source_id=%d\n", num_hops, my_id, destination_id, source_id);
#endif

	RREQ_Table_Append(source_id, rreq_id);

	uint8_t packet_arr[RREQ_PKT_LEN];

	packet_arr[0] = 0xFF;
	packet_arr[1] = 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RREQ(tosend, packet_arr + LORA_OFFSET);
	LoRa_SendData(packet_arr, RREQ_PKT_LEN);
	offset = 0;
}

void Mesh_Send_RREP(uint16_t next_hop_to_source, uint16_t rreq_source_id, uint16_t responder_id, uint8_t num_hops, uint32_t responder_sequence_number) {
#ifdef DEBUG
	printf("Building an RREP Packet\n");
#endif

	offset = 0;

	struct rrep_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = next_hop_to_source;
	tosend.source_id = rreq_source_id;
	tosend.responder_id = responder_id;
	tosend.num_hops = num_hops;
	tosend.responder_sequence_number = responder_sequence_number;

#ifdef DEBUG
	printf("\tnum_hops=%d rreq_source_id=%d responder_id=%d\n", num_hops, rreq_source_id, responder_id);
#endif

	uint8_t packet_arr[RREP_PKT_LEN];

	packet_arr[0] = (next_hop_to_source >> 8) & 0xFF;
	packet_arr[1] = next_hop_to_source & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RREP(tosend, packet_arr + LORA_OFFSET);
	LoRa_SendData(packet_arr, RREP_PKT_LEN);

	offset = 0;
}

void Mesh_Send_RERR(uint16_t receiver_id, uint16_t source_id, uint8_t num_hops) {

#ifdef DEBUG
	printf("Building an RERR Packet\n");
#endif

	offset = 0;

	struct rerr_packet tosend;


#ifdef DEBUG
	printf("RERR packet info\n");
#endif

	uint8_t packet_arr[RRER_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RERR(tosend, packet_arr + LORA_OFFSET);
	LoRa_SendData(packet_arr, RRER_PKT_LEN);

	offset = 0;
}

void Mesh_Send_Ping(uint16_t receiver_id, uint16_t destination_id, uint16_t source_id, uint8_t request_or_reply, uint32_t timestamp_ms) {

#ifdef DEBUG
	printf("Building a PING Packet\n");
#endif

	offset = 0;

	struct ping_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.request_or_reply = request_or_reply;
	tosend.timestamp_ms = timestamp_ms;

#ifdef DEBUG
	printf("\treceiver_id=%d destination_id=%d source_id=%d\n", receiver_id, destination_id, source_id);
#endif

	uint8_t packet_arr[PING_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_Ping(tosend, packet_arr + LORA_OFFSET);
	LoRa_SendData(packet_arr, PING_PKT_LEN);
	offset = 0;
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

void Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, DATA_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	memcpy(&packet_arr[offset], packet.packet_data, packet.data_length);
	offset += packet.data_length;

	End_Length_Count(packet_arr);

	offset = 0;
}

void Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREQ_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint32(packet_arr, packet.rreq_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint32(packet_arr, packet.source_sequence_number);

	End_Length_Count(packet_arr);

	offset = 0;
}

void Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREP_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint8(packet_arr, packet.num_hops);
	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.responder_id);
	Write_uint32(packet_arr, packet.responder_sequence_number);

	End_Length_Count(packet_arr);

	offset = 0;
}

void Format_Packet_RERR(struct rerr_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RERR_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint8(packet_arr, packet.num_unreachable_dests);

	for (uint8_t i = 0; i < packet.num_unreachable_dests; i++) {
		Write_uint16(packet_arr, packet.unreachable_dests[i].destination_id);
		Write_uint32(packet_arr, packet.unreachable_dests[i].destination_sequence_number);
	}

	End_Length_Count(packet_arr);

	offset = 0;
}

void Format_Packet_Ping(struct ping_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, PING_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint8(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.receiver_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint8(packet_arr, packet.request_or_reply);
	Write_uint32(packet_arr, packet.timestamp_ms);

	End_Length_Count(packet_arr);

	offset = 0;
}

struct data_packet Unpack_Packet_Data(uint8_t parr[], uint8_t data_length,
		uint8_t data[]) {
	offset = 0;

	struct data_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t payload_length =
	Read_uint8(parr);
	//	uint8_t reserved =
	Read_uint8(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.receiver_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);

	packet.data_length = data_length;
	if (data_length > 0 && data != NULL) {
		memcpy(data, &parr[offset], data_length);
		packet.packet_data = data;
	}

	offset = 0;

#ifdef DEBUG
	printf("\tData packet with:\n");
	printf(
			"\ttransmitter_id=%d receiver_id=%d destination_id=%d source_id=%d\n",
			packet.transmitter_id, packet.receiver_id,
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
	//	uint8_t reserved =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.rreq_id = Read_uint32(parr);
	packet.destination_id = Read_uint16(parr);
	packet.destination_sequence_number = Read_uint32(parr);
	packet.source_id = Read_uint16(parr);
	packet.source_sequence_number = Read_uint32(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tRREQ packet with:\n");
	printf(
			"\tnum_hops = %d transmitter_id=%d rreq_id=%" PRIu32 " destination_id=%d source_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.rreq_id,
			packet.destination_id, packet.source_id);
#endif

	return packet;
}

struct rrep_packet Unpack_Packet_RREP(uint8_t parr[]) {
	offset = 0;

	struct rrep_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved =
	Read_uint8(parr);

	packet.num_hops = Read_uint8(parr);
	packet.transmitter_id = Read_uint16(parr);
	packet.receiver_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.responder_id = Read_uint16(parr);
	packet.responder_sequence_number = Read_uint32(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tRREP packet with:\n");
	printf(
			"\tnum_hops = %d transmitter_id=%d receiver_id=%d responder_id=%d source_id=%d\n",
			packet.num_hops, packet.transmitter_id, packet.receiver_id,
			packet.responder_id, packet.source_id);
#endif
	return packet;
}

struct rerr_packet Unpack_Packet_RERR(uint8_t parr[]) {
	offset = 0;

	struct rerr_packet packet;

	//	uint8_t ptype =
		Read_uint8(parr);
	//	uint8_t payload_length =
		Read_uint8(parr);
	//	uint8_t reserved =
		Read_uint8(parr);

		packet.transmitter_id = Read_uint16(parr);
		packet.num_unreachable_dests = Read_uint8(parr);

		for (uint8_t i = 0; i < packet.num_unreachable_dests; i++) {
			packet.unreachable_dests[i].destination_id = Read_uint16(parr);
			packet.unreachable_dests[i].destination_sequence_number = Read_uint32(parr);
		}

		offset = 0;

	#ifdef DEBUG
		printf("\tRERR packet with:\n");
		printf(
				"\ttransmitter_id=%d num_unreachable_hosts=%d\n",
				packet.transmitter_id, packet.num_unreachable_dests);
	#endif
		return packet;
}

struct ping_packet Unpack_Packet_Ping(uint8_t parr[]) {
	offset = 0;

	struct ping_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved =
	Read_uint8(parr);

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
			"\ttransmitter_id=%d receiver_id=%d destination_id=%d source_id=%d request_or_reply:%d\n",
			packet.transmitter_id, packet.receiver_id,
			packet.destination_id, packet.source_id, packet.request_or_reply);
#endif
	return packet;
}

void Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength, uint8_t ptype) {
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
		Receive_Packet_Handler_RREQ(packet_data, plength);
		break;
	case RREP_PACKET:
		Receive_Packet_Handler_RREP(packet_data, plength);
		break;
	case RERR_PACKET:
		Receive_Packet_Handler_RERR(packet_data, plength);
		break;
	case DATA_PACKET:
		Receive_Packet_Handler_Data(packet_data, plength);
		break;
	case PING_PACKET:
		Receive_Packet_Handler_Ping(packet_data, plength);
		break;
	case INVALID_PACKET:
		Receive_Packet_Handler_Invalid(packet_data, plength);
		break;
	default:
		Receive_Packet_Handler_Invalid(packet_data, plength);
		break;
	}
}

void Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength) {
	volatile struct rreq_packet pkt;
	pkt = Unpack_Packet_RREQ(packet_data);

	if (RREQ_Table_Contains(pkt.source_id, pkt.rreq_id)) {
#ifdef DEBUG
		printf("Stale RREQ %" PRIu32 "\n", pkt.rreq_id);
#endif
		return;
	}

#ifdef DEBUG
	printf("Valid RREQ\n");
#endif
	RREQ_Table_Append(pkt.source_id, pkt.rreq_id);

	if (pkt.destination_id == 0) {
		printf("Received a Hello packet from node %d\n", pkt.source_id);
		Receive_Packet_Handler_Hello(pkt.source_id);
	}

	Update_Route_Table(pkt.source_id, pkt.source_sequence_number, pkt.num_hops, pkt.transmitter_id);

	if (pkt.destination_id == my_id) {
#ifdef DEBUG
		printf("Sending reply back to %d (through transmitter %d) since I am the destination\n",
				pkt.source_id, pkt.transmitter_id);
#endif
		Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id, my_id, 0,
				Increment_Sequence_Number());
		return;
	}
	int8_t route_idx = Route_Exists(pkt.destination_id);
	if (route_idx != -1 && Is_Fresher_Route(routing_table[route_idx].destination_sequence_number, pkt.destination_sequence_number)) {
#ifdef DEBUG
		printf("Sending reply back to %d by the info from my routing table\n",
				pkt.source_id);
#endif
		Mesh_Send_RREP(pkt.transmitter_id, pkt.source_id, pkt.destination_id,
				routing_table[route_idx].hop_count + 1,
				routing_table[route_idx].destination_sequence_number);
		return;
	}
#ifdef DEBUG
	printf("Forwarding RREQ further\n");
	uint32_t t1 = HAL_GetTick();
#endif
//	rand_delay(500, 600);
#ifdef DEBUG
	uint32_t t2 = HAL_GetTick();
	printf("Delay took %lu ms\n", t2 - t1);
#endif
	Mesh_Send_RREQ(pkt.destination_id, pkt.destination_sequence_number, pkt.source_id,
			pkt.source_sequence_number, pkt.num_hops + 1, pkt.rreq_id);
}

void Receive_Packet_Handler_Hello(uint16_t source_id) {
	for (uint8_t i = 0; i < route_table_entries; i++) {
		if (routing_table[i].next_hop_destination_id == source_id || routing_table[i].destination_id == source_id) {
			routing_table[i].expiration_time = 0;
			routing_table[i].destination_sequence_number = 0;
#ifdef DEBUG
			printf(" Invalidated route to ID %d through next-hop ID %d New TTL = %" PRIu32 " New sequence number = %" PRIu32 "\n",
					routing_table[i].destination_id, routing_table[i].next_hop_destination_id,
					routing_table[i].expiration_time, routing_table[i].destination_sequence_number);
#endif
		}
	}
}

void Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength) {
	volatile struct rrep_packet pkt;
	pkt = Unpack_Packet_RREP(packet_data);

	Update_Route_Table(pkt.responder_id, pkt.responder_sequence_number, pkt.num_hops, pkt.transmitter_id);

	if (pkt.source_id == my_id) {
#ifdef DEBUG
		printf("RREP for me\n");
		printf("Searching noroute table for blocked requests, with %d entries\n", pending_messages_table_entries);
#endif
		for (int i = 0; i < pending_messages_table_entries; i++) {
			if (pending_messages_table[i].destination_id == pkt.responder_id) {
#ifdef DEBUG
				printf("Found a matching entry in the noroute table\n");
				printf("\tdestination_id=%d receiver_id=%d transmitter_id=%d data_length=%d\n",
						pending_messages_table[i].destination_id,
						pkt.transmitter_id, my_id, pending_messages_table[i].data_length);
#endif
				if (pending_messages_table[i].data_length == 4 && memcmp(pending_messages_table[i].data, "ping", 4) == 0) {
					Mesh_Send_Ping(pkt.transmitter_id, pending_messages_table[i].destination_id, my_id, PING_REQUEST, Get_Timestamp());
				} else {
					Mesh_Send_Data(pending_messages_table[i].destination_id, pending_messages_table[i].data, pkt.transmitter_id, my_id, pending_messages_table[i].data_length);
				}
				pending_messages_table[i].destination_id = 0;
			}
		}
	} else {
		int8_t reverse_route_idx = Route_Exists(pkt.source_id);
		if (reverse_route_idx != -1) {
		Mesh_Send_RREP(routing_table[reverse_route_idx].next_hop_destination_id,
				pkt.source_id, pkt.responder_id, pkt.num_hops + 1,
				pkt.responder_sequence_number);
		}
		// In case we didn't receive a RREQ, but we know that transmitter did - therefore transmitter knows how to get to the destination
		Update_Route_Table(pkt.source_id, 0, pkt.num_hops + 1, pkt.transmitter_id);
	}
}

void Receive_Packet_Handler_RERR(uint8_t packet_data[], uint8_t plength) {
	volatile struct rerr_packet pkt;
	pkt = Unpack_Packet_RERR(packet_data);

	for (uint8_t i = 0; i < pkt.num_unreachable_dests; i++) {
		uint16_t dest_id = pkt.unreachable_dests[i].destination_id;
		int8_t route_idx = Route_Exists(dest_id);
		if (route_idx != -1 && Is_Fresher_Route(pkt.unreachable_dests[i].destination_sequence_number, routing_table[route_idx].destination_sequence_number) && routing_table[route_idx].next_hop_destination_id == pkt.transmitter_id)
			routing_table[route_idx].expiration_time = 0;
		// Forward RERR to precursors of this destination
	}
}

void Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength) {
	volatile struct data_packet pkt;
	pkt = Unpack_Packet_Data(packet_data, (plength - DATA_BASE_PKT_LEN), rx_data);

	if (pkt.destination_id == my_id) {
		DATA_RX_HANDLER(pkt);
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1) {
			Mesh_Send_Data(routing_table[route_idx].destination_id,
					pkt.packet_data, routing_table[route_idx].next_hop_destination_id,
					pkt.source_id, pkt.data_length);
		}
	}
}

void Receive_Packet_Handler_Ping(uint8_t packet_data[], uint8_t plength) {
	volatile struct ping_packet pkt;
	pkt = Unpack_Packet_Ping(packet_data);

	if (pkt.destination_id == my_id) {
		if (pkt.request_or_reply == PING_REQUEST) {
			Mesh_Send_Ping(pkt.transmitter_id, pkt.source_id, pkt.destination_id, PING_REPLY, pkt.timestamp_ms);
		} else {
			printf("[T] Received ping reply from node id:%d. Latency: %" PRIu32 "ms\n", pkt.source_id, Get_Timestamp() - pkt.timestamp_ms);
			return;
		}
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1) {
			Mesh_Send_Ping(routing_table[route_idx].next_hop_destination_id, routing_table[route_idx].destination_id, pkt.source_id, pkt.request_or_reply, pkt.timestamp_ms);
		}
	}

	Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
}


void Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength) {
#ifdef DEBUG
	printf("Invalid Packet Type (data[0] = %d)\n", packet_data[0]);
#endif
	return;
}

void DATA_RX_HANDLER(struct data_packet rx_pkt) {
	//Print the received data to the console
	printf("\n##########################################\n");
	printf("Node %d -> YOU\n", rx_pkt.source_id);
	for (int i = 0; i < rx_pkt.data_length; i++)
		printf("%c", rx_pkt.packet_data[i]);
	printf("\n##########################################\n");
}

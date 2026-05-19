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

void Mesh_Send_Data(
		uint16_t receiver_id,
		uint16_t source_id,
		uint16_t destination_id,
		uint32_t message_id,
		uint8_t TTL,
		uint8_t data_length,
		uint8_t *data,
//		uint8_t hop_count,
		bool is_originated
		) {
#ifdef DEBUG
	printf("Building a Data Packet\n");
#endif

	offset = 0;

	data_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.message_id = message_id;
	tosend.TTL = TTL;
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
	TX_Queue_Push(packet_arr, DATA_PKT_LEN + data_length, DATA_PACKET, destination_id, 200, message_id, 3, is_originated);
	offset = 0;
}

void Mesh_Send_RREQ(
		uint16_t source_id,
		uint16_t destination_id,
		uint32_t source_sequence_number,
		uint32_t destination_sequence_number,
		uint32_t rreq_id,
		uint8_t num_hops,
		bool is_originated
		) {

#ifdef DEBUG
	printf("Building an RREQ Packet\n");
	printf("%" PRIu32 "\n", rreq_id);
#endif

	offset = 0;

	rreq_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.source_id = source_id;
	tosend.destination_id = destination_id;
	tosend.source_sequence_number = source_sequence_number;
	tosend.destination_sequence_number = destination_sequence_number;
	tosend.rreq_id = rreq_id;
	tosend.hop_count = num_hops;

#ifdef DEBUG
	printf("\tnum_hops=%d transmitter_id=%d destination_id=%d source_id=%d\n", num_hops, my_id, destination_id, source_id);
#endif

	RREQ_Table_Append(source_id, rreq_id);

	uint8_t packet_arr[RREQ_PKT_LEN];

	packet_arr[0] = 0xFF;
	packet_arr[1] = 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RREQ(tosend, packet_arr + LORA_OFFSET);
	TX_Queue_Push(packet_arr, RREQ_PKT_LEN, RREQ_PACKET, destination_id, 255, 0, 0, is_originated);
	offset = 0;
}

void Mesh_Send_RREP(
		uint16_t next_hop_to_source,
		uint16_t rreq_source_id,
		uint16_t responder_id,
		uint32_t responder_sequence_number,
		uint8_t TTL,
		uint8_t num_hops,
		bool is_originated
		) {
#ifdef DEBUG
	printf("Building an RREP Packet\n");
#endif

	offset = 0;

	rrep_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.source_id = rreq_source_id;
	tosend.responder_id = responder_id;
	tosend.responder_sequence_number = responder_sequence_number;
	tosend.TTL = TTL;
	tosend.hop_count = num_hops;

#ifdef DEBUG
	printf("\tnum_hops=%d rreq_source_id=%d responder_id=%d\n", num_hops, rreq_source_id, responder_id);
#endif

	uint8_t packet_arr[RREP_PKT_LEN];

	packet_arr[0] = (next_hop_to_source >> 8) & 0xFF;
	packet_arr[1] = next_hop_to_source & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RREP(tosend, packet_arr + LORA_OFFSET);
	TX_Queue_Push(packet_arr, RREP_PKT_LEN, RREP_PACKET, rreq_source_id, 100, 0, 0, is_originated);

	offset = 0;
}

void Mesh_Send_RERR(
		uint16_t receiver_id,
		uint8_t dest_count,
		uint16_t *invalidated_dests,
		uint32_t *invalidated_seq_nums
		) {
#ifdef DEBUG
	printf("Building an RERR Packet\n");
#endif

	offset = 0;

	rerr_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.num_unreachable_dests = dest_count;

	if (dest_count > MAX_UNREACHABLE_DESTS) {
		dest_count = MAX_UNREACHABLE_DESTS;
#ifdef DEBUG
		printf("WARNING: RERR truncated to %d destinations\n", MAX_UNREACHABLE_DESTS);
#endif
	}

	for (uint8_t i = 0; i < dest_count; i++) {
		tosend.unreachable_dests[i].destination_id = invalidated_dests[i];
		tosend.unreachable_dests[i].destination_sequence_number = invalidated_seq_nums[i];
	}


#ifdef DEBUG
	printf("RERR packet info\n");
#endif

	uint8_t packet_arr[RRER_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_RERR(tosend, packet_arr + LORA_OFFSET);
	TX_Queue_Push(packet_arr, RRER_PKT_LEN, RERR_PACKET, receiver_id, 255, 0, 0, true);

	offset = 0;
}

void Mesh_Send_Ping(
		uint16_t receiver_id,
		uint16_t source_id,
		uint16_t destination_id,
		uint32_t message_id,
		uint8_t TTL,
		uint8_t request_or_reply,
		uint32_t timestamp_ms,
//		uint8_t hop_count,
		bool is_originated
		) {

#ifdef DEBUG
	printf("Building a PING Packet\n");
#endif

	offset = 0;

	ping_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.source_id = source_id;
	tosend.destination_id = destination_id;
	tosend.message_id = message_id;
	tosend.TTL = TTL;
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
	TX_Queue_Push(packet_arr, PING_PKT_LEN, PING_PACKET, destination_id, 180, message_id, 3, is_originated);
	offset = 0;
}

void Mesh_Send_ACK(
		uint16_t receiver_id,
		uint16_t source_id,
		uint16_t destination_id,
		uint8_t TTL,
		uint32_t message_id,
		bool is_originated
		) {
#ifdef DEBUG
	printf("Building an ACK Packet\n");
#endif

	offset = 0;

	ack_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.source_id = source_id;
	tosend.destination_id = destination_id;
	tosend.TTL = TTL;
	tosend.message_id = message_id;

#ifdef DEBUG
	printf("\treceiver_id=%d destination_id=%d message_id=%" PRIu32 "\n", receiver_id, destination_id, message_id);
#endif

	uint8_t packet_arr[ACK_PKT_LEN];

	packet_arr[0] = (receiver_id >> 8) & 0xFF;
	packet_arr[1] = receiver_id & 0xFF;
	packet_arr[2] = my_channel;

	Format_Packet_ACK(tosend, packet_arr + LORA_OFFSET);
	TX_Queue_Push(packet_arr, ACK_PKT_LEN, ACK_PACKET, destination_id, 254, 0, 0, is_originated);
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

void Format_Packet_Data(data_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, DATA_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.message_id);
	Write_uint8(packet_arr, packet.TTL);
	memcpy(&packet_arr[offset], packet.packet_data, packet.data_length);
	offset += packet.data_length;

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

void Format_Packet_RREQ(rreq_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREQ_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.rreq_id);
	Write_uint32(packet_arr, packet.source_sequence_number);
	Write_uint32(packet_arr, packet.destination_sequence_number);
	Write_uint8(packet_arr, packet.hop_count);

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

void Format_Packet_RREP(rrep_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RREP_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.responder_id);
	Write_uint32(packet_arr, packet.responder_sequence_number);
	Write_uint8(packet_arr, packet.TTL);
	Write_uint8(packet_arr, packet.hop_count);

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

void Format_Packet_RERR(rerr_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, RERR_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint8(packet_arr, packet.num_unreachable_dests);

	for (uint8_t i = 0; i < packet.num_unreachable_dests; i++) {
		Write_uint16(packet_arr, packet.unreachable_dests[i].destination_id);
		Write_uint32(packet_arr, packet.unreachable_dests[i].destination_sequence_number);
	}

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

void Format_Packet_Ping(ping_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, PING_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint32(packet_arr, packet.message_id);
	Write_uint8(packet_arr, packet.TTL);
	Write_uint8(packet_arr, packet.request_or_reply);
	Write_uint32(packet_arr, packet.timestamp_ms);

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

void Format_Packet_ACK(ack_packet packet, uint8_t packet_arr[]) {
	offset = 0;

	Write_uint8(packet_arr, ACK_PACKET);
	Write_uint8(packet_arr, Start_Length_Count());
	Write_uint16(packet_arr, 0);

	Write_uint16(packet_arr, my_id);
	Write_uint16(packet_arr, packet.source_id);
	Write_uint16(packet_arr, packet.destination_id);
	Write_uint8(packet_arr, packet.TTL);
	Write_uint32(packet_arr, packet.message_id);

	End_Length_Count(packet_arr);

    uint16_t crc = Calculate_CRC16(packet_arr, offset);
    Write_uint16(packet_arr, crc);

	offset = 0;
}

data_packet Unpack_Packet_Data(uint8_t parr[], uint8_t data_length,
		uint8_t data[]) {
	offset = 0;

	data_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t payload_length =
	Read_uint8(parr);
	//	uint8_t reserved =
	Read_uint16(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.message_id = Read_uint32(parr);
	packet.TTL = Read_uint8(parr);

	packet.data_length = data_length;
	if (data_length > 0 && data != NULL) {
		memcpy(data, &parr[offset], data_length);
		packet.packet_data = data;
	}

	offset = 0;

#ifdef DEBUG
	printf("\tData packet with:\n");
	printf(
			"\ttransmitter_id=%d destination_id=%d source_id=%d TTL=%d\n",
			packet.transmitter_id, packet.destination_id, packet.source_id, packet.TTL);
#endif
	return packet;
}

rreq_packet Unpack_Packet_RREQ(uint8_t parr[]) {
	offset = 0;

	rreq_packet packet;

	//	uint8_t ptype =
	Read_uint8(parr);
	//	uint8_t payload_length =
	Read_uint8(parr);
	//	uint8_t reserved =
	Read_uint16(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.rreq_id = Read_uint32(parr);
	packet.source_sequence_number = Read_uint32(parr);
	packet.destination_sequence_number = Read_uint32(parr);
	packet.hop_count = Read_uint8(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tRREQ packet with:\n");
	printf(
			"\tnum_hops = %d transmitter_id=%d rreq_id=%" PRIu32 " destination_id=%d source_id=%d\n",
			packet.hop_count, packet.transmitter_id, packet.rreq_id,
			packet.destination_id, packet.source_id);
#endif

	return packet;
}

rrep_packet Unpack_Packet_RREP(uint8_t parr[]) {
	offset = 0;

	rrep_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved =
	Read_uint16(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.responder_id = Read_uint16(parr);
	packet.responder_sequence_number = Read_uint32(parr);
	packet.TTL = Read_uint8(parr);
	packet.hop_count = Read_uint8(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tRREP packet with:\n");
	printf(
			"\tnum_hops = %d transmitter_id=%d responder_id=%d source_id=%d TTL=%d\n",
			packet.hop_count, packet.transmitter_id, packet.responder_id, packet.source_id, packet.TTL);
#endif
	return packet;
}

rerr_packet Unpack_Packet_RERR(uint8_t parr[]) {
	offset = 0;

	rerr_packet packet;

	//	uint8_t ptype =
		Read_uint8(parr);
	//	uint8_t payload_length =
		Read_uint8(parr);
	//	uint8_t reserved =
		Read_uint16(parr);

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
				"\ttransmitter_id=%d num_unreachable_hosts=%dn",
				packet.transmitter_id, packet.num_unreachable_dests);
	#endif
		return packet;
}

ping_packet Unpack_Packet_Ping(uint8_t parr[]) {
	offset = 0;

	ping_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved =
	Read_uint16(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.message_id = Read_uint32(parr);
	packet.TTL = Read_uint8(parr);
	packet.request_or_reply = Read_uint8(parr);
	packet.timestamp_ms = Read_uint32(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tPING packet with:\n");
	printf(
			"\ttransmitter_id=%d destination_id=%d source_id=%d request_or_reply:%d TTL=%d\n",
			packet.transmitter_id, packet.destination_id, packet.source_id, packet.request_or_reply, packet.TTL);
#endif
	return packet;
}

ack_packet Unpack_Packet_ACK(uint8_t parr[]) {
	offset = 0;

	ack_packet packet;

//	uint8_t ptype =
	Read_uint8(parr);
//	uint8_t payload_length =
	Read_uint8(parr);
//	uint8_t reserved =
	Read_uint16(parr);

	packet.transmitter_id = Read_uint16(parr);
	packet.source_id = Read_uint16(parr);
	packet.destination_id = Read_uint16(parr);
	packet.TTL = Read_uint8(parr);
	packet.message_id = Read_uint32(parr);

	offset = 0;

#ifdef DEBUG
	printf("\tACK packet with:\n");
	printf(
			"\ttransmitter_id=%d destination_id=%d message_id=%" PRIu32 "\n",
			packet.transmitter_id,
			packet.destination_id, packet.message_id);
#endif
	return packet;
}

#define CRC_POLYNOMIAL 0x1021

uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);

        for (uint8_t j = 0; j < 8; j++) {
        	if (crc & 0x8000) {
        		crc = (crc << 1) ^ CRC_POLYNOMIAL;
        	}
        	else {
        		crc = crc << 1;
        	}
        }
    }
    return crc & 0xFFFF;
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
	case RERR_PACKET:
		printf("\tPacket type is RERR\n");
		break;
	case DATA_PACKET:
		printf("\tPacket type is DATA\n");
		break;
	case PING_PACKET:
		printf("\tPacket type is PING\n");
		break;
	case ACK_PACKET:
		printf("\tPacket type is ACK\n");
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
	case ACK_PACKET:
		Receive_Packet_Handler_ACK(packet_data, plength);
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
	rreq_packet pkt;
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

	Update_Route_Table(pkt.source_id, pkt.source_sequence_number, pkt.hop_count, pkt.transmitter_id);

	if (pkt.destination_id == my_id) {
#ifdef DEBUG
		printf("Sending reply back to %d (through transmitter %d) since I am the destination\n",
				pkt.source_id, pkt.transmitter_id);
#endif
		Mesh_Send_RREP(pkt.transmitter_id,
				pkt.source_id,
				my_id,
				Increment_Sequence_Number(),
				MAX_HOPS_TTL,
				0,
				true);
		return;
	}
	int8_t route_idx = Route_Exists(pkt.destination_id);
	if (route_idx != -1 && Is_Fresher_Route(routing_table[route_idx].destination_sequence_number, pkt.destination_sequence_number)) {
#ifdef DEBUG
		printf("Sending reply back to %d by the info from my routing table\n",
				pkt.source_id);
#endif
		Mesh_Send_RREP(pkt.transmitter_id,
				pkt.source_id,
				pkt.destination_id,
				routing_table[route_idx].destination_sequence_number,
				MAX_HOPS_TTL,
				routing_table[route_idx].hop_count + 1,
				true);
		return;
	}
	if (pkt.hop_count != 255)
		Mesh_Send_RREQ(pkt.source_id,
				pkt.destination_id,
				pkt.source_sequence_number,
				pkt.destination_sequence_number,
				pkt.rreq_id,
				pkt.hop_count + 1,
				false);
}

void Receive_Packet_Handler_Hello(uint16_t source_id) {
	for (uint8_t i = 0; i < route_table_entries; i++) {
		if (routing_table[i].next_hop_destination_id == source_id || routing_table[i].destination_id == source_id) {
			routing_table[i].expiration_time = 0;
#ifdef DEBUG
			printf(" Invalidated route to ID %d through next-hop ID %d New TTL = %" PRIu32 "\n",
					routing_table[i].destination_id, routing_table[i].next_hop_destination_id,
					routing_table[i].expiration_time);
#endif
		}
	}
}

void Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength) {
	rrep_packet pkt;
	pkt = Unpack_Packet_RREP(packet_data);

	Update_Route_Table(pkt.responder_id, pkt.responder_sequence_number, pkt.hop_count, pkt.transmitter_id);

	Flush_Pending_Messages(pkt.responder_id);

	if (pkt.source_id == my_id) {
#ifdef DEBUG
		printf("RREP for me\n");
#endif
	} else {
		int8_t reverse_route_idx = Route_Exists(pkt.source_id);
		if (reverse_route_idx != -1 && pkt.TTL != 0) {
			Add_To_Precursor_List(reverse_route_idx, pkt.transmitter_id);

			Mesh_Send_RREP(routing_table[reverse_route_idx].next_hop_destination_id,
					pkt.source_id,
					pkt.responder_id,
					pkt.responder_sequence_number,
					pkt.TTL - 1,
					pkt.hop_count + 1,
					false);
		}
		// In case we didn't receive a RREQ, but we know that transmitter did - therefore transmitter knows how to get to the destination
		Update_Route_Table(pkt.source_id, 0, pkt.hop_count + 1, pkt.transmitter_id);
	}
}

void Receive_Packet_Handler_RERR(uint8_t packet_data[], uint8_t plength) {
	rerr_packet pkt;
	pkt = Unpack_Packet_RERR(packet_data);

	uint16_t invalidated_dests[pkt.num_unreachable_dests];
	uint8_t invalidated_count = 0;

	for (uint8_t i = 0; i < pkt.num_unreachable_dests; i++) {
		uint16_t dest_id = pkt.unreachable_dests[i].destination_id;
		int8_t route_idx = Route_Exists(dest_id);

		if (route_idx != -1
				&& Is_Fresher_Route(
						pkt.unreachable_dests[i].destination_sequence_number,
						routing_table[route_idx].destination_sequence_number)
				&& routing_table[route_idx].next_hop_destination_id
						== pkt.transmitter_id) {
#ifdef DEBUG
			printf("Invalidating route %d", routing_table[route_idx].destination_id);
#endif
			routing_table[route_idx].expiration_time = 0;
			invalidated_dests[invalidated_count] = dest_id;
			invalidated_count++;
		}
	}
	if (invalidated_count > 0) {
		Propagate_RERR_Upstream(invalidated_dests, invalidated_count);
		for (uint8_t i = 0; i < pkt.num_unreachable_dests; i++) {
			uint16_t unreachable = pkt.unreachable_dests[i].destination_id;
			TX_Queue_Drop_By_Destination(unreachable);
		}
	}
}

void Propagate_RERR_Upstream(uint16_t *invalidated_dests, uint8_t dest_count) {
	uint16_t precursor_nodes[MAX_PRECURSORS];
	uint8_t precursor_count = 0;

	//Collecting precursors for each invalidated route, making sure they are all unique

	for (uint8_t i = 0; i < dest_count; i++) {
		uint16_t dest_id = invalidated_dests[i];
		int8_t route_idx = Route_Exists(dest_id);

		if (route_idx != -1) {
			for (uint8_t j = 0; j < routing_table[route_idx].precursor_count; j++) {
				uint16_t precursor = routing_table[route_idx].precursor_list[j];

				uint8_t already_exists = 0;
				for (uint8_t k = 0; k < precursor_count; k++) {
					if (precursor_nodes[k] == precursor) {
						already_exists = 1;
						break;
					}
				}
				if (!already_exists && precursor_count < MAX_PRECURSORS) {
					precursor_nodes[precursor_count] = precursor;
					precursor_count++;
				}
			}
		}
	}

	//Looping over the array of precursors and sending each one an RERR with a list of what routes I've invalidated
	if (precursor_count > 0) {
#ifdef DEBUG
		printf("Propagating RERR to %d precursor(s)\n", precursor_count);
#endif
		uint32_t seq_nums[dest_count];
		for (uint8_t i = 0; i < dest_count; i++) {
			int8_t idx = Route_Exists(invalidated_dests[i]);
			if (idx != -1) {
				seq_nums[i] = routing_table[idx].destination_sequence_number;
			} else {
				seq_nums[i] = 0;
			}
		}

		for (uint8_t i = 0; i < precursor_count; i++) {
			Mesh_Send_RERR(precursor_nodes[i],
					dest_count,
					invalidated_dests,
					seq_nums);
#ifdef DEBUG
			printf("\tSent RERR to precursor %d\n", precursor_nodes[i]);
#endif
		}

	}

}

void Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength) {
	data_packet pkt;
	pkt = Unpack_Packet_Data(packet_data, (plength - (DATA_PKT_LEN - LORA_OFFSET)), rx_data);

	Mesh_Send_ACK(pkt.transmitter_id,
			my_id,
			pkt.transmitter_id,
			MAX_HOPS_TTL,
			pkt.message_id,
			true);
	if (pkt.destination_id == my_id) {
		Mesh_Send_ACK(pkt.transmitter_id,
							my_id,
							pkt.source_id,
							MAX_HOPS_TTL,
							pkt.message_id,
							true);
		DATA_RX_HANDLER(pkt);
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1 && pkt.TTL != 0) {
			Add_To_Precursor_List(route_idx, pkt.transmitter_id);
			uint16_t next_hop = routing_table[route_idx].next_hop_destination_id;

			Mesh_Send_Data(next_hop,
					pkt.source_id,
					pkt.destination_id,
					pkt.message_id,
					pkt.TTL - 1,
					pkt.data_length,
					pkt.packet_data,
					false);
		}
	}
}

void Receive_Packet_Handler_Ping(uint8_t packet_data[], uint8_t plength) {
	ping_packet pkt;
	pkt = Unpack_Packet_Ping(packet_data);

	Mesh_Send_ACK(pkt.transmitter_id,
			my_id,
			pkt.transmitter_id,
			MAX_HOPS_TTL,
			pkt.message_id,
			true);

	if (pkt.destination_id == my_id) {
		Mesh_Send_ACK(pkt.transmitter_id,
					my_id,
					pkt.source_id,
					MAX_HOPS_TTL,
					pkt.message_id,
					true);
		if (pkt.request_or_reply == PING_REQUEST) {
			Mesh_Send_Ping(pkt.transmitter_id,
					my_id,
					pkt.source_id,
					pkt.message_id,
					MAX_HOPS_TTL,
					PING_REPLY,
					pkt.timestamp_ms,
					true);
		} else {
			printf("[T] Received ping reply from node id:%d. Latency: %" PRIu32 "ms\n", pkt.source_id, Get_Timestamp() - pkt.timestamp_ms);
			return;
		}
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);
		Add_To_Precursor_List(route_idx, pkt.transmitter_id);
		uint16_t next_hop = routing_table[route_idx].next_hop_destination_id;

		if (route_idx != -1 && pkt.TTL != 0) {
			Mesh_Send_Ping(next_hop,
					pkt.source_id,
					pkt.destination_id,
					pkt.message_id,
					pkt.TTL - 1,
					pkt.request_or_reply,
					pkt.timestamp_ms,
					false);
		}
	}

	Update_Route_Table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
}

void Receive_Packet_Handler_ACK(uint8_t packet_data[], uint8_t plength) {
	ack_packet pkt;
	pkt = Unpack_Packet_ACK(packet_data);

	if (pkt.destination_id == my_id) {
		int8_t queue_idx = TX_Queue_Find_By_Message_ID(pkt.message_id);

		if (queue_idx != -1) {
			tx_queue_entry *orig_pkt = &tx_queue[queue_idx];

			if (pkt.source_id == orig_pkt->destination_id) {
				orig_pkt->ack_received_end_to_end = true;
				printf("E2E ACK received for message %" PRIu32 "\n", pkt.message_id);
			} else {
				orig_pkt->retry_count = 0;
				orig_pkt->last_tx_time_ms = 0;
				orig_pkt->ack_received_hop_by_hop = true;
				printf("Hop-by-hop ACK received for message %" PRIu32 "\n", pkt.message_id);
			}
		}
	} else {
		int8_t route_idx = Route_Exists(pkt.destination_id);

		if (route_idx != -1 && pkt.TTL != 0) {
			Add_To_Precursor_List(route_idx, pkt.transmitter_id);
			uint16_t next_hop = routing_table[route_idx].next_hop_destination_id;

			Mesh_Send_ACK(next_hop,
					pkt.source_id,
					pkt.destination_id,
					pkt.TTL - 1,
					pkt.message_id,
					false);
		}
	}
}


void Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength) {
#ifdef DEBUG
	printf("Invalid Packet Type (data[0] = %d)\n", packet_data[0]);
#endif
	return;
}

void DATA_RX_HANDLER(data_packet rx_pkt) {
	//Print the received data to the console
	printf("\n##########################################\n");
	printf("Node %d -> YOU\n", rx_pkt.source_id);
	for (int i = 0; i < rx_pkt.data_length; i++)
		printf("%c", rx_pkt.packet_data[i]);
	printf("\n##########################################\n");
}

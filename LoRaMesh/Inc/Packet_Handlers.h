#ifndef INC_PACKET_HANDLERS_H_
#define INC_PACKET_HANDLERS_H_

#include <stdbool.h>
#include "Packet_Formats.h"

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
		);

void Mesh_Send_RREQ(
		uint16_t source_id,
		uint16_t destination_id,
		uint32_t source_sequence_number,
		uint32_t destination_sequence_number,
		uint32_t rreq_id,
		uint8_t num_hops,
		bool is_originated
		);

void Mesh_Send_RREP(
		uint16_t next_hop_to_source,
		uint16_t rreq_source_id,
		uint16_t responder_id,
		uint32_t responder_sequence_number,
		uint8_t TTL,
		uint8_t num_hops,
		bool is_originated
		);

void Mesh_Send_RERR(
		uint16_t receiver_id,
		uint8_t dest_count,
		uint16_t *invalidated_dests,
		uint32_t *invalidated_seq_nums
		);

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
		);

void Mesh_Send_ACK(
		uint16_t receiver_id,
		uint16_t source_id,
		uint16_t destination_id,
		uint8_t TTL,
		uint32_t message_id,
		bool is_originated
		);

void Format_Packet_Data(data_packet packet, uint8_t packet_arr[]);
void Format_Packet_RREQ(rreq_packet packet, uint8_t packet_arr[]);
void Format_Packet_RREP(rrep_packet packet, uint8_t packet_arr[]);
void Format_Packet_RERR(rerr_packet packet, uint8_t packet_arr[]);
void Format_Packet_Ping(ping_packet packet, uint8_t packet_arr[]);
void Format_Packet_ACK(ack_packet packet, uint8_t packet_arr[]);
data_packet Unpack_Packet_DATA(uint8_t parr[]);
rreq_packet Unpack_Packet_RREQ(uint8_t parr[]);
rrep_packet Unpack_Packet_RREP(uint8_t parr[]);
rerr_packet Unpack_Packet_RERR(uint8_t parr[]);
ping_packet Unpack_Packet_Ping(uint8_t parr[]);
ack_packet Unpack_Packet_ACK(uint8_t parr[]);
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length);
void Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength, uint8_t ptype);
void Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Hello(uint16_t source_id);
void Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_RERR(uint8_t packet_data[], uint8_t plength);
void Propagate_RERR_Upstream(uint16_t *invalidated_dests, uint8_t dest_count);
void Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Ping(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_ACK(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength);
void DATA_RX_HANDLER(data_packet rx_pkt);

#endif /* INC_PACKET_HANDLERS_H_ */

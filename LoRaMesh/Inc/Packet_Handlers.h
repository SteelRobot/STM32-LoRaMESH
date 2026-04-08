#ifndef INC_PACKET_HANDLERS_H_
#define INC_PACKET_HANDLERS_H_

#include <stdbool.h>
#include "Packet_Formats.h"

void Mesh_Send_Data(uint16_t destination_id, uint8_t *data, uint16_t receiver_id, uint16_t source_id, uint8_t data_length);
void Mesh_Send_RREQ(uint16_t destination_id, uint16_t source_id, uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id);
void Mesh_Send_RREP(uint16_t receiver_id, uint16_t destination_id, uint16_t source_id, uint8_t num_hops, uint32_t dest_seq_num);
void Mesh_Send_Ping(uint16_t receiver_id, uint16_t destination_id, uint16_t source_id, uint8_t request_or_reply, uint32_t timestamp_ms);
void Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]);
void Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]);
void Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]);
void Format_Packet_RERR(struct rerr_packet packet, uint8_t packet_arr[]);
void Format_Packet_Ping(struct ping_packet packet, uint8_t packet_arr[]);
struct data_packet Unpack_Packet_DATA(uint8_t parr[]);
struct rreq_packet Unpack_Packet_RREQ(uint8_t parr[]);
struct rrep_packet Unpack_Packet_RREP(uint8_t parr[]);
struct rerr_packet Unpack_Packet_RERR(uint8_t parr[]);
struct ping_packet Unpack_Packet_Ping(uint8_t parr[]);
void Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength, uint8_t ptype);
void Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_RERR(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Ping(uint8_t packet_data[], uint8_t plength);
void Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength);
void DATA_RX_HANDLER(struct data_packet rx_pkt);

#endif /* INC_PACKET_HANDLERS_H_ */

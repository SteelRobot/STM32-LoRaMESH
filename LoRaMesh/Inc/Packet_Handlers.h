#ifndef INC_PACKET_HANDLERS_H_
#define INC_PACKET_HANDLERS_H_

#include <stdbool.h>
#include "Packet_Formats.h"

bool Mesh_Send_Data(uint16_t destination_id, uint32_t dest_seq_num, uint8_t *data, uint16_t receiver_id, uint8_t num_hops, uint16_t source_id, uint32_t source_sequence_number, uint8_t data_length);
bool Mesh_Send_RREQ(uint16_t destination_id, uint16_t source_id, uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id);
bool Mesh_Send_RREP(uint16_t receiver_id, uint16_t destination_id, uint16_t source_id, uint8_t num_hops, uint32_t dest_seq_num);
bool Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]);
bool Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]);
bool Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]);
bool Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength);
bool Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength);
bool Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength);
bool Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength);
bool Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength);
bool DATA_RX_HANDLER(struct data_packet rx_pkt);

#endif /* INC_PACKET_HANDLERS_H_ */

#ifndef INC_MESH_H_
#define INC_MESH_H_

#define UNICAST_TABLE_LENGTH 16
#define NOROUTE_QUEUE_MAX_ENTRIES 5
#define ROUTE_LIFETIME 100;
#define DEFAULT_ROUTE_EXPIRATION_TIME 1000

#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RERR_PACKET 3
#define INVALID_PACKET 0xF0

#define NO_ROUTE 0

#define DATA_BASE_PKT_LEN 2 + 2 + 2 + 4 + 4 + 1 + 1
#define RREQ_BASE_PKT_LEN 2 + 4 + 2 + 2 + 4 + 1
#define RREP_BASE_PKT_LEN 2 + 2 + 2 + 2 + 4 + 1

#define LORA_OFFSET 3
#define OPCODE_OFFSET 3

#define DATA_PREAMBLE_PKT_LEN DATA_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET
#define RREQ_PREAMBLE_PKT_LEN RREQ_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET
#define RREP_PREAMBLE_PKT_LEN RREP_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET

#define RREQ_TABLE_MAX_ENTRIES 10

struct data_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint32_t source_sequence_number;
	uint16_t source_id;
	uint8_t num_hops;
	uint8_t data_length; // Doesn't count for BASE_PKT_LEN
	uint8_t *packet_data;
};

struct rreq_packet {
	uint16_t transmitter_id;
	uint32_t rreq_id;
	uint16_t destination_id;
	uint16_t source_id;
	uint32_t source_sequence_number;
	uint8_t num_hops;
};

struct rrep_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t destination_id;
	uint16_t source_id;
	uint32_t destination_sequence_number;
	uint8_t num_hops;
};

struct unicast_route_table_entry {
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint32_t hop_count;
	uint16_t next_hop_destination_id;
	uint32_t expiration_time;
};

struct noroute_table_entry {
	uint16_t destination_id;
	uint8_t *data;
	uint8_t data_length;
};

void Mesh_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2,
		DMA_HandleTypeDef *hdma_usart1_rx);
uint8_t Mesh_Transmit(uint16_t destination_id, uint8_t data[],
		uint8_t data_length);
uint8_t Mesh_Send_Data(uint16_t destination_id, uint32_t dest_seq_num,
		uint8_t *data, uint16_t receiver_id, uint8_t num_hops, uint16_t source_id, uint32_t source_sequence_number,
		uint8_t data_length);
uint8_t Mesh_Send_RREQ(uint16_t destination_id, uint16_t source_id, uint32_t source_sequence_number,
		uint8_t num_hops, uint32_t rreq_id);
uint8_t Mesh_Send_RREP(uint16_t receiver_id, uint16_t destination_id,
		uint16_t source_id, uint8_t num_hops, uint32_t dest_seq_num);
uint8_t Format_Packet_Data(struct data_packet packet, uint8_t packet_arr[]);
uint8_t Format_Packet_RREQ(struct rreq_packet packet, uint8_t packet_arr[]);
uint8_t Format_Packet_RREP(struct rrep_packet packet, uint8_t packet_arr[]);
struct unicast_packet Unpack_Packet_Unicast(uint8_t parr[], uint8_t data_length,
		uint8_t data[]);
uint8_t Receive_Packet_Handler(uint8_t packet_data[], uint8_t plength);
uint8_t Receive_Packet_Handler_RREQ(uint8_t packet_data[], uint8_t plength);
uint8_t Receive_Packet_Handler_RREP(uint8_t packet_data[], uint8_t plength);
uint8_t Receive_Packet_Handler_Data(uint8_t packet_data[], uint8_t plength);
uint8_t Receive_Packet_Handler_Invalid(uint8_t packet_data[], uint8_t plength);
int8_t Route_Exists(uint16_t id);
uint8_t RREQ_Table_Contains(uint32_t rreq_id);
uint8_t RREQ_Table_Append(uint32_t rreq_id);
uint8_t Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num,
		uint8_t num_hops, uint16_t next_hop);
uint8_t Mesh_Send_Hello();
void rand_delay();
uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt);
uint8_t Increment_Sequence_Number(void);
uint8_t Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq);

#endif /* INC_MESH_H_ */

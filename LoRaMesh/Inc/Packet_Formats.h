#ifndef INC_PACKET_FORMATS_H_
#define INC_PACKET_FORMATS_H_

#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RERR_PACKET 3
#define INVALID_PACKET 0xF0

#define DATA_BASE_PKT_LEN 2 + 2 + 2 + 4 + 4 + 1 + 1
#define RREQ_BASE_PKT_LEN 2 + 4 + 2 + 2 + 4 + 1
#define RREP_BASE_PKT_LEN 2 + 2 + 2 + 2 + 4 + 1

#define LORA_OFFSET 3
#define OPCODE_OFFSET 3

#define DATA_PREAMBLE_PKT_LEN DATA_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET
#define RREQ_PREAMBLE_PKT_LEN RREQ_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET
#define RREP_PREAMBLE_PKT_LEN RREP_BASE_PKT_LEN + LORA_OFFSET + OPCODE_OFFSET

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

#endif /* INC_PACKET_FORMATS_H_ */

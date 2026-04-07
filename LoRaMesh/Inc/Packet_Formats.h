#ifndef INC_PACKET_FORMATS_H_
#define INC_PACKET_FORMATS_H_

#define DATA_PACKET 0
#define RREQ_PACKET 1
#define RREP_PACKET 2
#define RERR_PACKET 3
#define PING_PACKET 4
#define INVALID_PACKET 0xFF

#define PING_REQUEST 0
#define PING_REPLY 1

#define LORA_OFFSET 3
#define OPCODE_OFFSET 1
#define LENGTH_OFFSET 1
#define RESERVED_OFFSET 1

#define NUM_HOPS_LEN 1
#define TRANSMITTER_ID_LEN 2
#define RECEIVER_ID_LEN 2
#define DESTINATION_ID_LEN 2
#define SOURCE_ID_LEN 2
#define SEQUENCE_NUM_LEN 4
#define RREQ_ID_LEN 4
#define REQUEST_OR_REPLY_LEN 1
#define TIMESTAMP_LEN 4

#define DATA_BASE_PKT_LEN (OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + DESTINATION_ID_LEN + SOURCE_ID_LEN)

#define DATA_PKT_LEN (LORA_OFFSET + DATA_BASE_PKT_LEN)
#define RREQ_PKT_LEN (LORA_OFFSET + OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET + NUM_HOPS_LEN + TRANSMITTER_ID_LEN + RREQ_ID_LEN + DESTINATION_ID_LEN + SOURCE_ID_LEN + SEQUENCE_NUM_LEN)
#define RREP_PKT_LEN (LORA_OFFSET + OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET + NUM_HOPS_LEN + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + DESTINATION_ID_LEN + SEQUENCE_NUM_LEN + SOURCE_ID_LEN)
#define RRER_PKT_LEN (LORA_OFFSET + OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET)
#define PING_PKT_LEN (LORA_OFFSET + OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + DESTINATION_ID_LEN + SOURCE_ID_LEN + REQUEST_OR_REPLY_LEN + TIMESTAMP_LEN)

#define LENGTH_BYTE_POS 1

struct data_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t destination_id;
	uint16_t source_id;
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

struct rerr_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t destination_id;
	uint16_t source_id;
	uint32_t destination_sequence_number;
	uint8_t num_hops;
};

struct ping_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t destination_id;
	uint16_t source_id;
	uint32_t timestamp_ms;
	uint8_t request_or_reply;
};

#endif /* INC_PACKET_FORMATS_H_ */

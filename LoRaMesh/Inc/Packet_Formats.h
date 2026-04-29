#ifndef INC_PACKET_FORMATS_H_
#define INC_PACKET_FORMATS_H_

#define MAX_UNREACHABLE_DESTS 5

#define DATA_PACKET 1
#define RREQ_PACKET 2
#define RREP_PACKET 3
#define RERR_PACKET 4
#define PING_PACKET 5
#define ACK_PACKET 6
#define INVALID_PACKET 0xFF

#define VALID_OPCODES 6 // Amount of valid opcodes, used for data validation

#define PING_REQUEST 0
#define PING_REPLY 1

#define LORA_OFFSET 3
#define OPCODE_OFFSET 1
#define LENGTH_OFFSET 1
#define RESERVED_OFFSET 2

#define NUM_HOPS_LEN 1
#define TRANSMITTER_ID_LEN 2
#define RECEIVER_ID_LEN 2
#define DESTINATION_ID_LEN 2
#define SOURCE_ID_LEN 2
#define SEQUENCE_NUM_LEN 4
#define RREQ_ID_LEN 4
#define REQUEST_OR_REPLY_LEN 1
#define TIMESTAMP_LEN 4
#define NUM_UNREACHABLE_DESTS_LEN 1
#define UNREACHABLE_DESTS_LEN ((DESTINATION_ID_LEN + SEQUENCE_NUM_LEN) * MAX_UNREACHABLE_DESTS)
#define MESSAGE_ID_LEN 4
#define TTL_LEN 1
#define CRC_LEN 2

#define PKT_HEADER (LORA_OFFSET + OPCODE_OFFSET + LENGTH_OFFSET + RESERVED_OFFSET)
#define PKT_FOOTER (CRC_LEN)

#define DATA_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + SOURCE_ID_LEN + DESTINATION_ID_LEN + MESSAGE_ID_LEN + TTL_LEN + PKT_FOOTER)
#define RREQ_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + SOURCE_ID_LEN + DESTINATION_ID_LEN + RREQ_ID_LEN + SEQUENCE_NUM_LEN + SEQUENCE_NUM_LEN + NUM_HOPS_LEN + PKT_FOOTER)
#define RREP_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + SOURCE_ID_LEN + DESTINATION_ID_LEN + SEQUENCE_NUM_LEN + TTL_LEN + NUM_HOPS_LEN + PKT_FOOTER)
#define RRER_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + UNREACHABLE_DESTS_LEN + PKT_FOOTER)
#define PING_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + SOURCE_ID_LEN + DESTINATION_ID_LEN + MESSAGE_ID_LEN + TTL_LEN + REQUEST_OR_REPLY_LEN + TIMESTAMP_LEN + PKT_FOOTER)
#define ACK_PKT_LEN 	(PKT_HEADER + TRANSMITTER_ID_LEN + RECEIVER_ID_LEN + SOURCE_ID_LEN + DESTINATION_ID_LEN + MESSAGE_ID_LEN + PKT_FOOTER)

#define LENGTH_BYTE_POS 1

struct data_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t source_id;
	uint16_t destination_id;
	uint32_t message_id;
	uint8_t TTL;
	uint8_t data_length; // Doesn't count for BASE_PKT_LEN
	uint8_t *packet_data;
};

struct rreq_packet {
	uint16_t transmitter_id;
	uint16_t source_id;
	uint16_t destination_id;
	uint32_t rreq_id;
	uint32_t source_sequence_number;
	uint32_t destination_sequence_number;
	uint8_t hop_count;
};

struct rrep_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t source_id;
	uint16_t responder_id;
	uint32_t responder_sequence_number;
	uint8_t TTL;
	uint8_t hop_count;
};

struct rerr_packet {
	uint16_t transmitter_id;
	uint8_t num_unreachable_dests;
	struct {
		uint16_t destination_id;
		uint32_t destination_sequence_number;
	} unreachable_dests[MAX_UNREACHABLE_DESTS];
};

struct ping_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t source_id;
	uint16_t destination_id;
	uint32_t message_id;
	uint8_t TTL;
	uint8_t request_or_reply;
	uint32_t timestamp_ms;
};

struct ack_packet {
	uint16_t transmitter_id;
	uint16_t receiver_id;
	uint16_t source_id;
	uint16_t destination_id;
	uint32_t message_id;
};

#endif /* INC_PACKET_FORMATS_H_ */

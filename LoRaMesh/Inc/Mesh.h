#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdbool.h>

#define ROUTING_TABLE_LENGTH 16
#define MAX_PRECURSORS 10
#define MAX_PENDING_ACKS 10
#define PENDING_MESSAGES_TABLE_MAX_ENTRIES 5
#define RREQ_TABLE_MAX_ENTRIES 10
#define DEFAULT_ROUTE_EXPIRATION_TIME 10 // Seconds
#define DEFAULT_RREQ_EXPIRATION_TIME 10 // Seconds
#define MAX_PAYLOAD_SIZE 240
#define MAX_HOPS_TTL 10
#define MAX_RREQ_HOP_COUNT 255

#define ACK_TIMEOUT_MS 500
#define MAX_ACK_RETRIES 2

enum message_origin_type {
	GENERATED,
	FORWARDED
};

struct route_table_entry {
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint8_t hop_count;
	uint16_t next_hop_destination_id;
	uint32_t expiration_time;
	uint8_t precursor_count;
	uint16_t precursor_list[MAX_PRECURSORS];
};

struct pending_message_entry {
	uint16_t destination_id;
	uint16_t next_hop_destination_id;
	uint16_t source_id;
	uint8_t data_length;
	uint8_t data[MAX_PAYLOAD_SIZE];
	uint32_t message_id;
	enum message_origin_type origin;
	uint8_t TTL;
	uint32_t sent_time_ms;
	bool ack_received;
	uint8_t retries;
	uint8_t packet_type;
	uint8_t ping_request_or_reply; //for pings
};

struct rreq_table_entry {
	uint16_t source_id;
	uint32_t rreq_id;
	uint32_t expiration_time;
};

extern struct route_table_entry routing_table[ROUTING_TABLE_LENGTH];
extern struct pending_message_entry pending_messages_table[PENDING_MESSAGES_TABLE_MAX_ENTRIES];
extern uint8_t pending_messages_table_entries;
extern uint8_t route_table_entries;
extern RTC_TimeTypeDef currentTime;
extern RTC_DateTypeDef currentDate;

void Mesh_Init();
void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length);
void Pending_Messages_Table_Add(uint16_t next_hop_destination_id,
		uint16_t destination_id, uint16_t source_id, uint32_t message_id,
		uint8_t packet_type, uint8_t ttl, uint8_t ping_request_or_reply,
		uint8_t data[], uint8_t data_length, enum message_origin_type origin);
void Pending_Messages_Table_Add_Forwarded(uint16_t next_hop_destination_id,
		uint16_t destination_id, uint16_t source_id, uint32_t message_id,
		uint8_t packet_type, uint8_t TTL, uint8_t ping_request_or_reply,
		uint8_t data[], uint8_t data_length);
void Pending_Messages_Table_Add_Generated(uint16_t destination_id,
		uint8_t data[], uint8_t data_length);
void Pending_Messages_Table_Process(void);
int8_t Route_Exists(uint16_t id);
bool RREQ_Table_Contains(uint16_t source_id, uint32_t rreq_id);
void RREQ_Table_Append(uint16_t source_id, uint32_t rreq_id);
void Update_RREQ_Expiration(void);
void Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint16_t next_hop);
void Add_To_Precursor_List(uint8_t route_idx, uint16_t precursor_id);
void Mesh_Send_Hello(void);
void Generate_RREQ_ID(void);
uint32_t Increment_Sequence_Number(void);
uint32_t Get_Sequence_Number(void);
bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq);
void Update_Routes_Expiration(void);
uint8_t Get_Route_To_Replace(void);

#endif /* INC_MESH_H_ */

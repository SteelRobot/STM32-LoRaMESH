#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdbool.h>

#define ROUTING_TABLE_LENGTH 16
#define PENDING_MESSAGES_TABLE_MAX_ENTRIES 5
#define RREQ_TABLE_MAX_ENTRIES 10
#define DEFAULT_ROUTE_EXPIRATION_TIME 600 // Seconds
#define DEFAULT_RREQ_EXPIRATION_TIME 10 // Seconds
#define MAX_PAYLOAD_SIZE 1024

struct route_table_entry {
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint8_t hop_count;
	uint16_t next_hop_destination_id;
	uint32_t expiration_time;
};

struct pending_message {
	uint16_t destination_id;
	uint8_t data_length;
	uint8_t data[MAX_PAYLOAD_SIZE];
};

struct rreq_table_entry {
	uint16_t source_id;
	uint32_t rreq_id;
	uint32_t expiration_time;
};

extern struct route_table_entry routing_table[ROUTING_TABLE_LENGTH];
extern struct pending_message pending_messages_table[PENDING_MESSAGES_TABLE_MAX_ENTRIES];
extern uint8_t pending_messages_table_entries;
extern uint8_t route_table_entries;
extern RTC_TimeTypeDef currentTime;
extern RTC_DateTypeDef currentDate;

void Mesh_Init();
void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length);
void Noroute_Table_Add(uint16_t destination_id, uint8_t data[], uint8_t data_length);
int8_t Route_Exists(uint16_t id);
bool RREQ_Table_Contains(uint16_t source_id, uint32_t rreq_id);
void RREQ_Table_Append(uint16_t source_id, uint32_t rreq_id);
void Update_RREQ_Expiration(void);
void Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint16_t next_hop);
void Mesh_Send_Hello(void);
void Generate_RREQ_ID(void);
uint32_t Increment_Sequence_Number(void);
uint32_t Get_Sequence_Number(void);
bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq);
void Update_Routes_Expiration(void);
uint8_t Get_Route_To_Replace(void);

#endif /* INC_MESH_H_ */

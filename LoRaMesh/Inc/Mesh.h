#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdbool.h>

#define UNICAST_TABLE_LENGTH 16
#define NOROUTE_QUEUE_MAX_ENTRIES 5
#define RREQ_TABLE_MAX_ENTRIES 10
#define DEFAULT_ROUTE_EXPIRATION_TIME 1000

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

extern struct unicast_route_table_entry unicast_route_table[UNICAST_TABLE_LENGTH];
extern struct noroute_table_entry noroute_table[NOROUTE_QUEUE_MAX_ENTRIES];
extern uint32_t unicast_entries;
extern uint8_t noroute_table_entries;

void Mesh_Init();
bool Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length);
int8_t Route_Exists(uint16_t id);
bool RREQ_Table_Contains(uint32_t rreq_id);
bool RREQ_Table_Append(uint32_t rreq_id);
bool Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint16_t next_hop);
bool Mesh_Send_Hello();
void Generate_RREQ_ID(void);
uint8_t Increment_Sequence_Number(void);
bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq);

#endif /* INC_MESH_H_ */

#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdbool.h>

#define ROUTING_TABLE_LENGTH 16
#define MAX_PRECURSORS 10
#define MAX_PENDING_ACKS 10
#define TX_QUEUE_SIZE 16
#define RREQ_TABLE_MAX_ENTRIES 10
#define DEFAULT_ROUTE_EXPIRATION_TIME 300 // Seconds
#define DEFAULT_RREQ_EXPIRATION_TIME 10 // Seconds
#define MAX_PAYLOAD_SIZE 240
#define MAX_HOPS_TTL 10
#define MAX_RREQ_HOP_COUNT 255

#define PACKET_RETRY_TIME_MS 4000
#define PACKET_RETRY_AMOUNT 3

struct route_table_entry {
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint8_t hop_count;
	uint16_t next_hop_destination_id;
	uint32_t expiration_time;
	uint8_t precursor_count;
	uint16_t precursor_list[MAX_PRECURSORS];
};

struct rreq_table_entry {
	uint16_t source_id;
	uint32_t rreq_id;
	uint32_t expiration_time;
};

struct tx_queue_entry {
    uint8_t packet_data[MAX_PAYLOAD_SIZE];
    uint16_t packet_size;
    uint8_t packet_type;
    uint8_t priority;
    uint32_t timestamp_ms;
    uint32_t message_id;
    uint8_t retry_count;
    uint8_t max_retries;
    uint32_t last_tx_time_ms;
    bool needs_retry;
    uint8_t ack_received;
};

extern struct tx_queue_entry tx_queue[TX_QUEUE_SIZE];

extern struct route_table_entry routing_table[ROUTING_TABLE_LENGTH];
extern uint8_t route_table_entries;
extern RTC_TimeTypeDef currentTime;
extern RTC_DateTypeDef currentDate;

void Mesh_Init();
void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length);
int8_t Route_Exists(uint16_t id);
bool RREQ_Table_Contains(uint16_t source_id, uint32_t rreq_id);
void RREQ_Table_Append(uint16_t source_id, uint32_t rreq_id);
void Update_RREQ_Expiration(void);
bool TX_Queue_Push(uint8_t *serialized_data, uint16_t size, uint8_t type, uint8_t priority, uint32_t message_id, uint8_t max_retries);
void TX_Queue_Pop(void);
struct tx_queue_entry* TX_Queue_Peek(void);
void TX_Queue_Handle_ACK(uint32_t message_id);
void TX_Queue_Check_Timeouts(uint32_t timeout_ms);
void TX_Queue_Process(void);
void Update_Route_Table(uint16_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint16_t next_hop);
void Add_To_Precursor_List(uint8_t route_idx, uint16_t precursor_id);
void Mesh_Send_Hello(void);
uint32_t Generate_RREQ_ID(void);
uint32_t Generate_Packet_ID(void);
uint32_t Increment_Sequence_Number(void);
uint32_t Get_Sequence_Number(void);
bool Is_Fresher_Route(uint32_t new_seq, uint32_t old_seq);
void Update_Routes_Expiration(void);
uint8_t Get_Route_To_Replace(void);

#endif /* INC_MESH_H_ */

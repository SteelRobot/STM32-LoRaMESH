#ifndef INC_MESH_H_
#define INC_MESH_H_

#include <stdbool.h>

#define ROUTING_TABLE_LENGTH 16
#define MAX_PRECURSORS 10
#define MAX_PENDING_ACKS 10
#define TX_QUEUE_SIZE 16
#define MAX_PENDING_MESSAGES 10
#define PENDING_MSG_TIMEOUT_MS 15000 // ms
#define RREQ_TABLE_MAX_ENTRIES 10
#define DEFAULT_ROUTE_EXPIRATION_TIME 300 // Seconds
#define DEFAULT_RREQ_EXPIRATION_TIME 10 // Seconds
#define MAX_PAYLOAD_SIZE 240
#define MAX_HOPS_TTL 10
#define MAX_RREQ_HOP_COUNT 255

#define PACKET_RETRY_TIME_MS 4000
#define PACKET_RETRY_AMOUNT 3
#define PRIORITY_PACKETS_AMOUNT 4

typedef struct {
	uint16_t destination_id;
	uint32_t destination_sequence_number;
	uint8_t hop_count;
	uint16_t next_hop_destination_id;
	uint32_t expiration_time;
	uint8_t precursor_count;
	uint16_t precursor_list[MAX_PRECURSORS];
} route_table_entry;

typedef struct {
	uint16_t source_id;
	uint32_t rreq_id;
	uint32_t expiration_time;
} rreq_table_entry;

typedef struct {
    uint16_t destination_id;
    uint8_t data[MAX_PAYLOAD_SIZE];
    uint8_t data_length;
    bool is_ping;
    uint32_t timestamp_ms;
    bool in_use;
} pending_message;

typedef struct {
    uint8_t packet_data[MAX_PAYLOAD_SIZE];
    uint16_t packet_size;
    uint8_t packet_type;
    uint16_t destination_id;
    uint8_t hop_count;
    uint8_t priority;
    uint32_t message_id;
    uint8_t retry_count;
    uint8_t max_retries;
    uint32_t last_tx_time_ms;
    bool ready_to_send;

    bool ack_received_hop_by_hop;
    bool ack_received_end_to_end;

    bool is_originated;
} tx_queue_entry;

extern tx_queue_entry tx_queue[TX_QUEUE_SIZE];

extern route_table_entry routing_table[ROUTING_TABLE_LENGTH];
extern uint8_t route_table_entries;
extern RTC_TimeTypeDef currentTime;
extern RTC_DateTypeDef currentDate;

void Mesh_Init();
void Mesh_Transmit(uint16_t destination_id, uint8_t data[], uint8_t data_length);
int8_t Route_Exists(uint16_t id);
bool RREQ_Table_Contains(uint16_t source_id, uint32_t rreq_id);
void RREQ_Table_Append(uint16_t source_id, uint32_t rreq_id);
void Update_RREQ_Expiration(void);
void Buffer_Pending_Message(uint16_t destination_id, uint8_t *data, uint8_t data_length, bool is_ping);
void Flush_Pending_Messages(uint16_t destination_id);
bool TX_Queue_Push(uint8_t *serialized_data, uint16_t size,
                   uint8_t type, uint16_t destination_id,
//				   uint8_t hop_count,
				   bool is_originated,
				   uint8_t priority, uint32_t message_id, uint8_t max_retries);
void TX_Queue_Pop(void);
tx_queue_entry* TX_Queue_Peek(void);
int8_t TX_Queue_Find_By_Message_ID(uint32_t message_id);
void TX_Queue_Drop_By_Destination(uint16_t destination_id);
void TX_Queue_Handle_Packet_Failure(tx_queue_entry *pkt);
void TX_Queue_Check_Timeouts(void);
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

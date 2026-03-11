#ifndef INC_MESH_H_
#define INC_MESH_H_

#include "main.h"
#include "LoRa.h"

#define MAX_NETWORK_NODES 64
#define MAX_ROUTE_HOPS 10
// Max amount of bytes per packet
#define MTU_SIZE 200

struct NodeAddress {
    uint16_t node_id;
};

// For caching the route
struct RouteEntry {
    struct NodeAddress destination;
    uint8_t hop_count;
    struct NodeAddress next_hop[MAX_ROUTE_HOPS];
    bool is_fresh; // To check if another RREQ needs to be done
//    uint32_t timestamp; // Route discovery moment, might implement, but will need to add clocks/timers
};

struct RREQ_Header {
	uint16_t address;
	uint8_t channel;
    uint8_t op_code;
    struct NodeAddress sender_id;
    struct NodeAddress dest_id;
    uint8_t hop_count;
//    uint32_t timestamp; // To prevent infinite re-broadcasting, might implement, but will need to add clocks/timers
    uint8_t sequence_num; // Also to prevent loops
    struct NodeAddress path[MAX_ROUTE_HOPS];
};

struct RREP_Header {
	uint16_t address;
	uint8_t channel;
    uint8_t op_code;
    struct NodeAddress dest_id; // Who will send the reply
    struct NodeAddress sender_id; // Who requested the reply
    uint8_t hop_count;
    struct NodeAddress path[MAX_ROUTE_HOPS];
};

struct DataPacket {
	uint16_t address;
	uint8_t channel;
    uint8_t op_code;
    uint16_t total_length;
    struct NodeAddress sender_id;
    struct NodeAddress dest_id;
    uint8_t data[MTU_SIZE];
};

void Mesh_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);

void Mesh_ParsePacket(void);

// 1 if the route is found and cached
// 0 if not
bool Mesh_DiscoverRoute(struct NodeAddress target);

void Mesh_SendData(struct NodeAddress target, uint8_t data, uint8_t data_size);

// Forwarding a packet through an already known route, without broadcasting RREQ
void Mesh_ForwardPacket(struct DataPacket* packet);

void Mesh_ReportBrokenRoute(struct NodeAddress broken_link_node, struct NodeAddress target);

#endif /* INC_MESH_H_ */

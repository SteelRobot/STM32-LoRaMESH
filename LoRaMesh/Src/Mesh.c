#include "main.h"
#include "LoRa.h"
#include "Mesh.h"
#include <string.h>

// Dynamic Source Routing
// No periodic beaconing
// Node floods the network with a Route REQuest (RREQ)
// Needed node replies with a Route REPly (RREP) on the first ping that it got

#define RREQ_OPCODE 0x01
#define RREP_OPCODE 0x02
#define DATA_OPCODE 0x00

struct NodeAddress current_node;
struct RouteEntry routing_table[MAX_NETWORK_NODES];
static uint32_t request_sequence = 0;
static uint8_t channel;

void Mesh_Init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, DMA_HandleTypeDef *hdma_usart1_rx) {

	LoRa_Init(huart1, huart2, hdma_usart1_rx);

	current_node.node_id = current_node_address;
	channel = current_node_channel;

	for (uint32_t i = 0; i < MAX_NETWORK_NODES; i++) {
		// Zero-ing the table
	        routing_table[i].is_fresh = false;
	        routing_table[i].hop_count = 0;
	    }


    struct RouteEntry self_route;
    self_route.destination = current_node;
    self_route.hop_count = 0;
    self_route.next_hop[0] = current_node;
    self_route.is_fresh = true;

    routing_table[0] = self_route;
}

void Mesh_SendData(struct NodeAddress target, uint8_t data, uint8_t data_size) {
	// Add checks of routing table
	// If not in table - initiate route discovery

	struct DataPacket data_packet;
	memset(&data_packet, 0, sizeof(data_packet));

	data_packet.address = target.node_id; // Actually should be next hop in the routing table
	data_packet.channel = channel;
	data_packet.op_code = 0x00;
	data_packet.total_length = sizeof(data_packet);
	data_packet.sender_id = current_node;
	data_packet.dest_id = target;
	memcpy(&data_packet.data, &data, data_size);

	LoRa_SendData((uint8_t *) &data_packet, (uint8_t) sizeof(data_packet));
}

bool Mesh_DiscoverRoute(struct NodeAddress target) {
	request_sequence++;

	struct RREQ_Header rreq;
	memset(&rreq, 0, sizeof(rreq));

	rreq.address = 0xFFFF;
	rreq.channel = channel;
	rreq.op_code = RREQ_OPCODE; // Define 0x01 as RREQ opcode in your LoRa app layer
	rreq.sender_id = current_node;
	rreq.dest_id = target;
	rreq.hop_count = 0; // Source starts with 0 hops to reach itself (will be incremented by others)
	rreq.sequence_num = request_sequence;
	memset(rreq.path, 0, sizeof(rreq.path));

	LoRa_SendData((uint8_t *) &rreq, sizeof(rreq));

	HAL_Delay(5000); // Waiting for reply



	return true;
}

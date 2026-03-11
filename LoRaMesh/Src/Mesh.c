#include "main.h"
#include "Mesh.h"

// Distance Vector - Router Advertisement
struct Node {
	uint8_t Destination;
	uint8_t Next;
	uint8_t Metric;
	uint8_t Sequence;
};

// Dynamic Source Routing
// No periodic beaconing
// Node floods the network with a Route REQuest (RREQ)

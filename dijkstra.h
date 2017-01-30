#include "define.h"
#include "object.h"
#include "malloc.h"

struct _INFO_DIJKSTRA *dijkstra(struct _INFO_NODE_ARC adr_node_arc,int start,int end);
struct _INFO_DIJKSTRA *dijkstra_time(struct _INFO_NODE_ARC adr_node_arc,int start,int end);
struct _INFO_DIJKSTRA *dijkstra_weight(struct _INFO_NODE_ARC adr_node_arc,int start,int end);


#include "define.h"
#include "object.h"
#include "malloc.h"
#include "dijkstra.h"
#include <stdio.h>
#include <stdlib.h>


void *car_move_plan_normal(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int start_node,int end_node,int sim_step);

void *car_move_plan_startrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int end_node,int sim_step);

void *car_move_plan_endrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int start_node,int sim_step);

void *car_move_plan_allrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int sim_step);

void *add_car_move_plan(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int init_car_num,int sim_step);

double K_Vel(double k);

_INFO_CAR_MOVING *car_move(struct _INFO_NODE_ARC adr_node_arc);
//_INFO_CAR_MOVING *car_move_test(struct _INFO_NODE_ARC adr_node_arc);
_INFO_CAR_MOVING *car_move_forever_SD(struct _INFO_NODE_ARC adr_node_arc);
_INFO_CAR_MOVING *car_move_forever_ST(struct _INFO_NODE_ARC adr_node_arc);
_INFO_CAR_MOVING *car_move_forever_RIS(struct _INFO_NODE_ARC adr_node_arc);


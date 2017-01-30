#ifndef OBJECT
#define OBJECT
#include "define.h"

struct CAR_STAT {   // status of car(s).
    bool arrived;
    short link;     // どのリンク上にいるか？
    short block;
    double pos; //そのリンク上のどの場所にいるかを，リンク原点からの距離で表現
    double speed;    // 速度，原点から見て離れる方向だと正値．逆だと負値．停止中だとゼロ．
};

struct CAR_MOVING_HISTORY {
    bool arrived;
    int car_ID; // 車両のID(番号)
    volatile double gain_distance;
    int birth_step; // 当該車両が生成されたシミュレーションステップ番号
    struct CAR_STAT history[MAX_PERIOD_of_CAR_EXISTENCE];   // 当該車両の履歴
};

struct CAR_MOVING_PLAN{
    int plan_length;
    int plan[MAX_PLAN_LENGTH];
    int plan_arc[MAX_PLAN_LENGTH];
};

struct NODE{ // 道路ネットワークのノードの構造定義
    int node_ID;// ノードのID
    double x,y; // ノードの座標値（単位：メートル(m)）
    int connected_arc_num;  //本ノードに接続されるアークの数
    int connected_arc[MAX_CONNECTED_ARC_NUM];// 本ノードに接続されるアークの集合
    int connected_node[MAX_CONNECTED_ARC_NUM];// 本ノードに１本のアークを介して直接的に接続されるノードの集合
};

struct BLOCK{
    int block_car_num;//本ノードにいる車輌数
    double block_length;//アークの長さ
    double k;//アークの密度
};

struct ARC{  // 道路ネットワークのアークの構造定義
    int arc_ID;// アークのID
    int ori_node,des_node;// 始点・終点のノードのID
    double length;// アークの長さ（単位：メートル(m)）
    double weight;
    double expect_time;
    int num_blocks;//アークのブラック数
    struct BLOCK block[MAX_BLOCK_NUM][MAX_SIM_STEP];
};

struct _INFO_NODE_ARC{
    struct NODE *node;
    struct ARC *arc;
    int num_node;
    int num_arc;
    int length;
};

struct _INFO_DIJKSTRA{
    int shortest_path_length;
    int shortest_distance;
    int path_plan[MAX_PLAN_LENGTH];
};

struct _INFO_CAR_MOVING{
    int num_car_moving;
    struct CAR_MOVING_HISTORY *car_moving_history;
    struct CAR_MOVING_PLAN *car_moving_plan;
};

#endif // OBJECT

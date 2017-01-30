#include "car.h"


CAR_MOVING_HISTORY *car_moving_history;
CAR_MOVING_PLAN *car_moving_plan;
_INFO_DIJKSTRA *_info_dijkstra;
_INFO_CAR_MOVING *_info_car_moving;

int car_step[MAX_CAR_NUM];

double K_Vel(double k){
  double V;

  V = 500.0/36.0*(1.0-(100.0/14.0)*k);

  if(V <= 0)
    V = 0.1;

  return V;
}

void *car_move_plan_normal(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int start_node,int end_node,int sim_step){

	_info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));

	_info_dijkstra = dijkstra(adr_node_arc,start_node,end_node);

	    //pass the paln to car
        (_info_car_moving.car_moving_plan+init_start_carID)->plan_length = _info_dijkstra->shortest_path_length;
        //pass the node_path to car
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID)->plan_length; j++){
        	(_info_car_moving.car_moving_plan+init_start_carID)->plan[j] = _info_dijkstra->path_plan[j];  //--node		
        }
        //pss the arc_path to car
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID)->plan_length - 1) ; k++){
            for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc_num;n++){
              if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n]))->des_node == 	(_info_car_moving.car_moving_plan+init_start_carID)->plan[k+1])
              (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n];
             }
        }

      //導入した位置を修正する(導入した前には-1です)
    	(_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].link = (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[0];
    	(_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].pos = 0;

      //経路情報のメモリをfree
      free(_info_dijkstra);
      return car_moving_history;

}

void *car_move_plan_startrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int end_node,int sim_step){

  _info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));
  int start_node_rand = rand()%(adr_node_arc.num_node+1);
  _info_dijkstra = dijkstra(adr_node_arc,start_node_rand,end_node);

      //pass the paln to car
        (_info_car_moving.car_moving_plan+init_start_carID)->plan_length = _info_dijkstra->shortest_path_length;
        //pass the node_path to car
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID)->plan_length; j++){
          (_info_car_moving.car_moving_plan+init_start_carID)->plan[j] = _info_dijkstra->path_plan[j];  //--node    
        }
        //pss the arc_path to car
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID)->plan_length - 1) ; k++){
            for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc_num;n++){
              if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n]))->des_node ==   (_info_car_moving.car_moving_plan+init_start_carID)->plan[k+1])
              (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n];
             }
        }

        //導入した位置を修正する(導入した前には-1です)
      (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].link = (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[0];
      (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].pos = 0;

    //経路情報のメモリをfree
    free(_info_dijkstra);
    return car_moving_history;
}

void *car_move_plan_endrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int start_node,int sim_step){

    _info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));

    //経路情報を書き込み    ----    OK
        int end_node;
        end_node = rand()%(adr_node_arc.num_node);
        //printf("start at %d ,end at %d\n",start_node,end_node);
        while(start_node == end_node){
            end_node = rand()%adr_node_arc.num_node;
        }
       (_info_car_moving.car_moving_history+init_start_carID)->birth_step = sim_step;
        _info_dijkstra = dijkstra(adr_node_arc,start_node,end_node);

    //pass the paln to car
        (_info_car_moving.car_moving_plan+init_start_carID)->plan_length = _info_dijkstra->shortest_path_length;
        //pass the node_path to car
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID)->plan_length; j++){
          (_info_car_moving.car_moving_plan+init_start_carID)->plan[j] = _info_dijkstra->path_plan[j];  //--node    
        //printf("start %d end %d ,%d step is %d\n",start_node,end_node,j, _info_dijkstra->path_plan[j]);
        }
        //pass the arc_path to car
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID)->plan_length - 1) ; k++){
            for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc_num;n++){
              if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n]))->des_node ==   (_info_car_moving.car_moving_plan+init_start_carID)->plan[k+1])
              (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n];
             }
        }

    //導入した位置を修正する(導入した前には-1です)
    (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].link = (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[0];
    (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].pos = 0;

    //経路情報のメモリをfree
    free(_info_dijkstra);
    return car_moving_history;
}

void *car_move_plan_allrand(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int sim_step){

    _info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));

    //経路情報を書き込み    ----    OK
        int start_node,end_node;
         start_node = rand()%((adr_node_arc.num_node)+1);
         end_node = rand()%((adr_node_arc.num_node)+1);

        //start_node = 0;
        //end_node = 99;


        //printf("start at %d ,end at %d\n",start_node,end_node);
        while(start_node == end_node){
            end_node = rand()%adr_node_arc.num_node;
        }
	     (_info_car_moving.car_moving_history+init_start_carID)->birth_step = sim_step;
        _info_dijkstra = dijkstra(adr_node_arc,start_node,end_node);

    //pass the paln to car
        (_info_car_moving.car_moving_plan+init_start_carID)->plan_length = _info_dijkstra->shortest_path_length;
        //pass the node_path to car
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID)->plan_length; j++){
        	(_info_car_moving.car_moving_plan+init_start_carID)->plan[j] = _info_dijkstra->path_plan[j];  //--node		
        //printf("start %d end %d ,%d step is %d\n",start_node,end_node,j, _info_dijkstra->path_plan[j]);
        }
        //pass the arc_path to car
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID)->plan_length - 1) ; k++){
            for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc_num;n++){
              if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n]))->des_node == 	(_info_car_moving.car_moving_plan+init_start_carID)->plan[k+1])
              (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID)->plan[k])->connected_arc[n];
             }
        }

    //導入した位置を修正する(導入した前には-1です)
    (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].link = (_info_car_moving.car_moving_plan+init_start_carID)->plan_arc[0];
    (_info_car_moving.car_moving_history+init_start_carID)->history[sim_step].pos = 0;

    //経路情報のメモリをfree
    free(_info_dijkstra);
    return car_moving_history;
}

void *add_car_move_plan(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int init_car_num,int sim_step){

    _info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));

    //経路情報を書き込み    ----    OK
    for(int i = 0 ; i < init_car_num ; i++){
        int start_node,end_node;
        start_node = rand()%adr_node_arc.num_node;
        end_node = rand()%adr_node_arc.num_node;
        while(start_node == end_node){
            end_node = rand()%adr_node_arc.num_node;
        }
    (_info_car_moving.car_moving_history+init_start_carID+i)->birth_step = sim_step;
        _info_dijkstra = dijkstra(adr_node_arc,start_node,end_node);
        (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length = _info_dijkstra->shortest_path_length;
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length ; j++){
        (_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j] = _info_dijkstra->path_plan[j];
	//	printf("car_moving_plan %d plan[%d] = %d\n",i,j,(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j]);    //-----node
        }
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length - 1) ; k++){
            for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc_num;n++){
              if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc[n]))->des_node == 	(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k+1])
              (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc[n];

             }
        }
    }

    //導入した位置を修正する(導入した前には-1です)
    for(int i = 0 ; i < init_car_num ; i++){
    (_info_car_moving.car_moving_history+init_start_carID+i)->history[0].link = (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_arc[0];
    (_info_car_moving.car_moving_history+init_start_carID+i)->history[0].pos = 0;
    }

    //経路を表示
	//    for(int i = 0 ; i < init_car_num ; i++){
	//        printf("Car %d have %d steps to end,",init_start_carID+i,(_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length);
	//        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length+1; j++)
	//            printf("%d ",(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j]);
	//        printf("\n");
	//    }

    //経路情報のメモリをfree
    free(_info_dijkstra);
    return car_moving_history;
}


_INFO_CAR_MOVING *car_move(struct _INFO_NODE_ARC adr_node_arc)
{


    car_moving_history = (CAR_MOVING_HISTORY*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    car_moving_plan =(CAR_MOVING_PLAN*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    _info_car_moving = (_INFO_CAR_MOVING*)malloc(sizeof(_INFO_CAR_MOVING));

    _info_car_moving->num_car_moving = 0;
    _info_car_moving->car_moving_history = car_moving_history;
    _info_car_moving->car_moving_plan = car_moving_plan;

    //初期化
    int sim_step = 0;
        //CAR_MOVING_HISTORYを初期化   ---   OK
         for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          car_step[i] = 0 ;
             (car_moving_history+i)->arrived = false;
         (car_moving_history+i)->car_ID = i ;
             (car_moving_history+i)->birth_step = -1; //-1 means not exist
            for(int j = 0 ; j < MAX_PLAN_LENGTH;j++){
                (car_moving_history+i)->history[j].link = -1;   //-1 means not exist
                (car_moving_history+i)->history[j].pos = -1;
                (car_moving_history+i)->history[j].speed=10;
            }
         }


       //CAR_MOVING_PLANを初期化    ---   OK
     for(int i = 0 ; i < MAX_CAR_NUM ; i++){
         (car_moving_plan+i)->plan_length = 0;
            for(int j = 0 ; j <MAX_PLAN_LENGTH;j++){
                (car_moving_plan+i)->plan[j] = -1;
            (car_moving_plan+i)->plan_arc[j] = -1;
            }
         }
    printf("初期化完了\n");



    //main part of moving
    while(sim_step <= MAX_SIM_STEP){

        //Add cars to simulation
          if(_info_car_moving->num_car_moving+10 <= MAX_CAR_NUM){
              if(sim_step % 5 == 0){
      //        printf("Added cars in %d steps\n",sim_step);
                add_car_move_plan(adr_node_arc,*_info_car_moving,_info_car_moving->num_car_moving,10,sim_step);
                _info_car_moving->num_car_moving+=10;
              }
          }

        //Dijkstraで計算した経路で移動する
        for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
              if(!(car_moving_history+i)->arrived){
               (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].link = (car_moving_plan+i)->plan_arc[car_step[i]];
               (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)+1].pos = (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].pos+(car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].speed;
              }
                }


        //到着を判断する
               for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
                 if(!(car_moving_history+i)->arrived){
                   if((car_step[i] >= ((car_moving_plan+i)->plan_length - 1))){
                    (car_moving_history+i)->arrived = true;
             //       printf("Car %d was arried in %d steps.\n",i,sim_step);
                   }
                 }
               }


        //次のアークに移動したを判断する
               for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
            if(!(car_moving_history+i)->arrived){
            int move_step = sim_step - ((car_moving_history+i)->birth_step);
            //printf("move_step = %d \n",move_step);
                    if((car_moving_history+i)->history[move_step+1].pos >= (adr_node_arc.arc+(car_moving_history+i)->history[move_step].link)->length){
            int now_pos = (car_moving_history+i)->history[move_step].pos;
            int now_link = (car_moving_history+i)->history[move_step].link;
            (car_moving_history+i)->history[move_step+1].pos -= (adr_node_arc.arc+now_link)->length;
                    car_step[i]++;
                    }
                   }
            }

        //履歴を表示  --  OK
	//        for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
	//            if((car_moving_history+i)->arrived)
	//                break;
	//        printf("Car %d in link %d pos %d at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,sim_step);
	//        }


        sim_step++;

      //  printf("step = %d\n",sim_step);
    }
	//    //特定車の履歴を表示
	//    int display_carID = 3929;
	//    printf("start_node = %d end_node = %d \n",(_info_car_moving->car_moving_plan+display_carID)->plan[0],(_info_car_moving->car_moving_plan+display_carID)->plan[(_info_car_moving->car_moving_plan+display_carID)->plan_length-1]);
	//   printf("Car %d have %d steps\n",display_carID,(_info_car_moving->car_moving_plan+display_carID)->plan_length);
	//    for(int i = 0 ; i <MAX_PLAN_LENGTH;i++){
	//        printf("Car %d in link %d pos %d at %d steps\n",display_carID,(_info_car_moving->car_moving_history+display_carID)->history[i].link,(_info_car_moving->car_moving_history+display_carID)->history[i].pos,i+(_info_car_moving->car_moving_history+display_carID)->birth_step);
	//   }
    printf("num_car = %d \nDone!\n",_info_car_moving->num_car_moving);
    return _info_car_moving;
}


_INFO_CAR_MOVING *car_move_forever_SD(struct _INFO_NODE_ARC adr_node_arc){
    
    //the memory of moving
    int memory = 0;
    
    //メモリを定義する
    car_moving_history = (CAR_MOVING_HISTORY*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    
    //check memory of car_memory_history
    if(car_moving_history){
     // printf("car_moving_history = %p\n",car_moving_history );
      printf("sizeof(car_moving_history) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY) );
      printf("checked memory of car_moving_history,sizeof(car_moving_history) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    }
    else{
      printf("failed in check memory of car_moving_history\n");
      printf("End\n");
      exit(0);
    }

    car_moving_plan =(CAR_MOVING_PLAN*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    //check memory of car_moving_plan
    if(car_moving_plan){
     // printf("car_moving_plan = %p\n",car_moving_plan );
      printf("sizeof(car_moving_plan) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(car_moving_plan) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    }
    else{
      printf("failed in check memory of car_moving_plan\n");
      printf("End\n");
      exit(0);
    }

    _info_car_moving = (_INFO_CAR_MOVING*)malloc(sizeof(_INFO_CAR_MOVING));
    //check memory of _info_car_moving
    if(_info_car_moving){
     // printf("car_moving_plan = %p\n",car_moving_plan );
     // printf("sizeof(_info_car_moving) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(_info_car_moving) is %d bytes\n",sizeof(_INFO_CAR_MOVING));
    }
    else{
      printf("failed in check memory of _info_car_moving\n");
      printf("End\n");
      exit(0);
    }
    //printf("_info_car_moving = %p\n", _info_car_moving);


    _info_car_moving->num_car_moving = 0;
    _info_car_moving->car_moving_history = car_moving_history;
    _info_car_moving->car_moving_plan = car_moving_plan;

    //初期化
    int sim_step = 0;
        //CAR_MOVING_HISTORYを初期化   ---   OK
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
              car_step[i] = 0 ;
             (car_moving_history+i)->arrived = false;
             (car_moving_history+i)->car_ID = i ;
             (car_moving_history+i)->gain_distance = 0.0;
             (car_moving_history+i)->birth_step = 0; //-1 means not exist
            for(int j = 0 ; j < MAX_PERIOD_of_CAR_EXISTENCE;j++){
                (car_moving_history+i)->history[j].link  = -1;   //-1 means not exist
                (car_moving_history+i)->history[j].pos   = -1.0;
                (car_moving_history+i)->history[j].block = 0;
                (car_moving_history+i)->history[j].speed = -1.0;
            }
         }

       //CAR_MOVING_PLANを初期化    ---   OK
     	for(int i = 0 ; i < MAX_CAR_NUM ; i++){
         (car_moving_plan+i)->plan_length = 0;
            for(int j = 0 ; j <MAX_PLAN_LENGTH;j++){
                (car_moving_plan+i)->plan[j] = -1;
                (car_moving_plan+i)->plan_arc[j] = -1;
            }
      }
    printf("車の情報初期化完了\n");

    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    // 	printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
    printf("最大車輌数(%d)で実験する\n",MAX_CAR_NUM);
    _info_car_moving->num_car_moving = MAX_CAR_NUM;
    //main part of moving
    while(sim_step < MAX_SIM_STEP ){
        
        //Init the plan of moving
              if(sim_step == 0){
                for(int i = 0 ; i < MAX_CAR_NUM;i++){
                    car_move_plan_allrand(adr_node_arc,*_info_car_moving,i,sim_step);
                }
              }

        //到着を判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if((car_moving_history+i)->history[sim_step].link == -1){
                if(car_step[i] >= ((car_moving_plan+i)->plan_length-1)){
                  // (car_moving_history+i)->arrived = true;
                    // int now_link = (car_moving_history+i)->history[sim_step-1].link;
                    // int loop_start_node = (adr_node_arc.arc+now_link)->des_node;
                   // car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,loop_start_node,sim_step);
                  //  (car_moving_history+i)->arrived = false;
                  int last_node = (adr_node_arc.arc+(car_moving_history+i)->history[sim_step-1].link)->des_node;
                  car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,last_node,sim_step);
                    car_step[i]=0;
                   }
            }
        }



        //Dijkstraで計算した経路で移動する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
               (car_moving_history+i)->history[sim_step].link = (car_moving_plan+i)->plan_arc[car_step[i]];
               int now_link = (car_moving_history+i)->history[sim_step].link;
               int now_pos  = (car_moving_history+i)->history[sim_step].pos;

               double block_pos = (adr_node_arc.arc+now_link)->block[0][sim_step].block_length;
               
               //detect in whick block
                int loop_num_blocks = 0;
                while(loop_num_blocks < (adr_node_arc.arc+now_link)->num_blocks){                  
                  if(now_pos < block_pos)
                    break;
                  else
                      {
                       loop_num_blocks++;
                       block_pos +=(adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_length;
                      }
                }
                //show the result
                (car_moving_history+i)->history[sim_step].block = loop_num_blocks;
                //printf("car %d in link %d block %d\n", i , now_link ,loop_num_blocks );
                //add the car to the block
                (adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_car_num++;               
        }

        //set speed to every moving cars
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          int c_link = (car_moving_history+i)->history[sim_step].link;
          int c_block = (car_moving_history+i)->history[sim_step].block;
          (car_moving_history+i)->history[sim_step].speed = K_Vel((adr_node_arc.arc+c_link)->block[c_block][sim_step].block_car_num / (adr_node_arc.arc+c_link)->block[c_block][sim_step].block_length);
         // printf("Car %d 's speed = %f , in %d steps.\n", i,(car_moving_history+i)->history[sim_step].speed,sim_step);
        } 


        //move to next pos       
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          (car_moving_history+i)->history[sim_step+1].pos = (car_moving_history+i)->history[sim_step].pos + TIME_UNIT * (car_moving_history+i)->history[sim_step].speed;
        }


        //gain distance
          for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            double gain_distance_step = 0;
            gain_distance_step = (TIME_UNIT * (car_moving_history+i)->history[sim_step-1].speed);
          //  // printf("before,gain_distance = %f  speed = %f\n", (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step-1].speed );
          //printf("before add ,gain_distance = %f , TIME_UNIT = %d speed = %f sim_step = %d\n", (car_moving_history+i)->gain_distance,TIME_UNIT,(car_moving_history+i)->history[sim_step-1].speed,sim_step);
           (car_moving_history+i)->gain_distance +=  gain_distance_step ;                  
             printf("After add, gain_distance = %f \n", (car_moving_history+i)->gain_distance);
           //printf("After,gain_distance = %f  speed = %f in %d steps\n", (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step-1].speed,sim_step );

          // //  if((car_moving_history+i)->gain_distance < 0)
          // //  printf("car%d 's gain_distance = %f ,speed = %f,in %d steps\n", i , (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step].speed,sim_step );
           }

         //total gain distance
         if(sim_step <= MAX_SIM_STEP ){
          double total_distance = 0;
          for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            total_distance += (car_moving_history+i)->gain_distance;
            printf("car %d gain_distance = %f\n",i, (car_moving_history+i)->gain_distance);
          }
          //printf("total_distance = %f km\n",total_distance / 1000);
          //printf("the average velocity = %f km/h\n", (total_distance / (MAX_CAR_NUM * (MAX_SIM_STEP-2)*10))*3.6 );
         }  
       

        //show the state of every moving cars 
        if(sim_step < MAX_SIM_STEP -2){       
          for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            //printf("car%d was in link %d pos %f block %d gain_distance = %f(m) speed = %f(m/s),at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,(car_moving_history+i)->history[sim_step].block,(car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step].speed,sim_step);
          }
        }

        //次のアークに移動したを判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if((car_moving_history+i)->history[sim_step+1].pos >= (adr_node_arc.arc+(car_moving_history+i)->history[sim_step].link)->length){

                int now_pos = (car_moving_history+i)->history[sim_step].pos;
                int now_link = (car_moving_history+i)->history[sim_step].link;

                (car_moving_history+i)->history[sim_step+1].pos -= (adr_node_arc.arc+now_link)->length;
                car_step[i]++;
                }
        }

        //履歴を表示  --  OK
	       // for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
	       //     if((car_moving_history+i)->arrived)
	       //         break;
	       // printf("Car %d in link %d pos %d at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,sim_step);
	       // }
        sim_step++;
    }
        //特定車の履歴を表示
        int display_carID = 5;
       //  printf("start_node = %d end_node = %d \n",(_info_car_moving->car_moving_plan+display_carID)->plan[0],(_info_car_moving->car_moving_plan+display_carID)->plan[(_info_car_moving->car_moving_plan+display_carID)->plan_length-1]);
       // printf("Car %d have %d steps\n",display_carID,(_info_car_moving->car_moving_plan+display_carID)->plan_length);
       //  for(int i = 0 ; i <MAX_PERIOD_of_CAR_EXISTENCE;i++){
       //      printf("Car %d in link %d pos %d at %d steps\n",display_carID,(_info_car_moving->car_moving_history+display_carID)->history[i].link,(_info_car_moving->car_moving_history+display_carID)->history[i].pos,i);
       // }
    

    //check the num of car in every step --- OK
    // for(int loop_step = 0 ; loop_step < sim_step ; loop_step++){
      // int cars_num = 0 ;
      // for(int loop_arc = 0 ; loop_arc < adr_node_arc.num_arc ; loop_arc++)
      //   for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+loop_arc)->num_blocks ; loop_block++){
      //     cars_num += ((adr_node_arc.arc+loop_arc)->block[loop_block][loop_step].block_car_num);
      //   }
      //   printf("cars_num = %d at %d steps\n",cars_num,loop_step );
    //   }


     //watch the total_num in everystep --  OK
     for(int k = 0 ; k < sim_step ; k++){
      int total_num = 0;
     for(int i = 0 ; i < adr_node_arc.num_arc ; i++)
       for(int j = 0 ; j < (adr_node_arc.arc+i)->num_blocks ; j++){
        total_num += (adr_node_arc.arc+i)->block[j][k].block_car_num;
       }
       printf("total_num = %d in %d steps\n",total_num , k );
     }
      

    //watch the Block[0] of Arc5 
    // for(int i = 0 ; i < sim_step ; i++)
    //   printf("Arc 5 block 0 have %d cars in %d steps\n", (adr_node_arc.arc+5)->block[0][i].block_car_num,i);
     printf("Done\n");
           
    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    // 	printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
}

_INFO_CAR_MOVING *car_move_forever_ST(struct _INFO_NODE_ARC adr_node_arc){
    
    //the memory of moving
    int memory = 0;
    
    //メモリを定義する
    car_moving_history = (CAR_MOVING_HISTORY*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    
    //check memory of car_memory_history
    if(car_moving_history){
     // printf("car_moving_history = %p\n",car_moving_history );
      printf("sizeof(car_moving_history) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY) );
      printf("checked memory of car_moving_history,sizeof(car_moving_history) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    }
    else{
      printf("failed in check memory of car_moving_history\n");
      printf("End\n");
      exit(0);
    }

    car_moving_plan =(CAR_MOVING_PLAN*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    //check memory of car_moving_plan
    if(car_moving_plan){
     // printf("car_moving_plan = %p\n",car_moving_plan );
      printf("sizeof(car_moving_plan) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(car_moving_plan) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    }
    else{
      printf("failed in check memory of car_moving_plan\n");
      printf("End\n");
      exit(0);
    }

    _info_car_moving = (_INFO_CAR_MOVING*)malloc(sizeof(_INFO_CAR_MOVING));
    //check memory of _info_car_moving
    if(_info_car_moving){
     // printf("car_moving_plan = %p\n",car_moving_plan );
     // printf("sizeof(_info_car_moving) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(_info_car_moving) is %d bytes\n",sizeof(_INFO_CAR_MOVING));
    }
    else{
      printf("failed in check memory of _info_car_moving\n");
      printf("End\n");
      exit(0);
    }
    //printf("_info_car_moving = %p\n", _info_car_moving);


    _info_car_moving->num_car_moving = 0;
    _info_car_moving->car_moving_history = car_moving_history;
    _info_car_moving->car_moving_plan = car_moving_plan;

    //初期化
    int sim_step = 0;
        //CAR_MOVING_HISTORYを初期化   ---   OK
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
              car_step[i] = 0 ;
             (car_moving_history+i)->arrived = false;
             (car_moving_history+i)->car_ID = i ;
             (car_moving_history+i)->gain_distance = 0.0;
             (car_moving_history+i)->birth_step = 0; //-1 means not exist
            for(int j = 0 ; j < MAX_PERIOD_of_CAR_EXISTENCE;j++){
                (car_moving_history+i)->history[j].link  = -1;   //-1 means not exist
                (car_moving_history+i)->history[j].pos   = -1.0;
                (car_moving_history+i)->history[j].block = 0;
                (car_moving_history+i)->history[j].speed = -1.0;
            }
         }

       //CAR_MOVING_PLANを初期化    ---   OK
      for(int i = 0 ; i < MAX_CAR_NUM ; i++){
         (car_moving_plan+i)->plan_length = 0;
            for(int j = 0 ; j <MAX_PLAN_LENGTH;j++){
                (car_moving_plan+i)->plan[j] = -1;
                (car_moving_plan+i)->plan_arc[j] = -1;
            }
      }
    printf("車の情報初期化完了\n");

    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    //  printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
    printf("最大車輌数(%d)で実験する\n",MAX_CAR_NUM);
    _info_car_moving->num_car_moving = MAX_CAR_NUM;
    //main part of moving
    while(sim_step < MAX_SIM_STEP ){
        
        //Init the plan of moving
              if(sim_step == 0){
                for(int i = 0 ; i < MAX_CAR_NUM;i++){
                    car_move_plan_allrand(adr_node_arc,*_info_car_moving,i,sim_step);
                }
              }

        //到着を判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if(car_step[i] >= ((car_moving_plan+i)->plan_length-1)){
                if((car_moving_history+i)->history[sim_step].link == -1){
                  // (car_moving_history+i)->arrived = true;
                    // int now_link = (car_moving_history+i)->history[sim_step-1].link;
                    // int loop_start_node = (adr_node_arc.arc+now_link)->des_node;
                   // car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,loop_start_node,sim_step);
                  //  (car_moving_history+i)->arrived = false;
                  int last_node = (adr_node_arc.arc+(car_moving_history+i)->history[sim_step-1].link)->des_node;
                  car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,last_node,sim_step);
                    car_step[i]=0;
                   }
            }
        }



        //Dijkstraで計算した経路で移動する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
               (car_moving_history+i)->history[sim_step].link = (car_moving_plan+i)->plan_arc[car_step[i]];
               int now_link = (car_moving_history+i)->history[sim_step].link;
               int now_pos  = (car_moving_history+i)->history[sim_step].pos;

               double block_pos = (adr_node_arc.arc+now_link)->block[0][sim_step].block_length;
               
               //detect in whick block
                int loop_num_blocks = 0;
                while(loop_num_blocks < (adr_node_arc.arc+now_link)->num_blocks){                  
                  if(now_pos < block_pos)
                    break;
                  else
                      {
                       loop_num_blocks++;
                       block_pos +=(adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_length;
                      }
                }
                //show the result
                (car_moving_history+i)->history[sim_step].block = loop_num_blocks;
                //printf("car %d in link %d block %d\n", i , now_link ,loop_num_blocks );
                //add the car to the block
                (adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_car_num++;               
        }

        //set speed to every moving cars
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          int c_link = (car_moving_history+i)->history[sim_step].link;
          int c_block = (car_moving_history+i)->history[sim_step].block;
          (car_moving_history+i)->history[sim_step].speed = K_Vel((adr_node_arc.arc+c_link)->block[c_block][sim_step].block_car_num / (adr_node_arc.arc+c_link)->block[c_block][sim_step].block_length);
         // printf("Car %d 's speed = %f , in %d steps.\n", i,(car_moving_history+i)->history[sim_step].speed,sim_step);
        } 

        //set density to every block
        if(sim_step < MAX_SIM_STEP - 2){
        for(int loop_arc = 0 ; loop_arc < adr_node_arc.num_arc ; loop_arc++)
          for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+loop_arc)->num_blocks ; loop_block++){
           (adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].k = (double)(adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].block_car_num/(adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].block_length;
          //  printf("the density of arc[%d].block[%d][%d] = %f \n", loop_arc,loop_block,sim_step,(adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].k);
           }
        }

        //calcualate the expect time of each link
        for(int loop_arc = 0 ; loop_arc < adr_node_arc.num_arc ; loop_arc++){
          (adr_node_arc.arc+loop_arc)->expect_time = 0.0;
          for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+loop_arc)->num_blocks ; loop_block++){
            (adr_node_arc.arc+loop_arc)->expect_time += (adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].block_length / K_Vel( (adr_node_arc.arc+loop_arc)->block[loop_block][sim_step].k);
           }
           //printf("the expect_time of arc[%d] = %f \n", loop_arc,(adr_node_arc.arc+loop_arc)->expect_time);

        }

        //check the density and expect_time
        // int watch_arc = 5;
        // printf("arc %d 's expect_time = %f(s)\n", watch_arc , (adr_node_arc.arc+watch_arc)->expect_time);
        // for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+watch_arc)->num_blocks ; loop_block++){
        //   printf("the block%d's expect_time = %f(s)\n",loop_block,(adr_node_arc.arc+watch_arc)->block[loop_block][sim_step].block_length/K_Vel((adr_node_arc.arc+watch_arc)->block[loop_block][sim_step].k) );
        // }

        //move to next pos       
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          (car_moving_history+i)->history[sim_step+1].pos = (car_moving_history+i)->history[sim_step].pos + TIME_UNIT * (car_moving_history+i)->history[sim_step].speed;
        }


        //gain distance
          for(int i = 0 ; i < MAX_CAR_NUM ; i++){
          //  // printf("before,gain_distance = %f  speed = %f\n", (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step-1].speed );
          //  printf("before add ,gain_distance = %f , TIME_UNIT = %d speed = %f sim_step = %d\n", (car_moving_history+i)->gain_distance,TIME_UNIT,(car_moving_history+i)->history[sim_step-1].speed,sim_step);
           (car_moving_history+i)->gain_distance += (TIME_UNIT * (car_moving_history+i)->history[sim_step-1].speed);                   
          //   //printf("After add, gain_distance = %f \n", (car_moving_history+i)->gain_distance);
          // printf("After,gain_distance = %f  speed = %f in %d steps\n", (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step-1].speed,sim_step );

          // //  if((car_moving_history+i)->gain_distance < 0)
          // //  printf("car%d 's gain_distance = %f ,speed = %f,in %d steps\n", i , (car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step].speed,sim_step );
           }

         //total gain distance
         if(sim_step == MAX_SIM_STEP - 2){
          double total_distance;
          for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            total_distance += (car_moving_history+i)->gain_distance;
            printf("car %d gain_distance = %f\n",i, (car_moving_history+i)->gain_distance);
          }
          printf("total_distance = %f\n",total_distance );
         }  
       


        

        //show the state of every moving cars        
        // for(int i = 0 ; i < MAX_CAR_NUM ; i++){
        //   printf("car%d was in link %d pos %f block %d gain_distance = %f(m) speed = %f(m/s),at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,(car_moving_history+i)->history[sim_step].block,(car_moving_history+i)->gain_distance,(car_moving_history+i)->history[sim_step].speed,sim_step);
        // }

        //次のアークに移動したを判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if((car_moving_history+i)->history[sim_step+1].pos >= (adr_node_arc.arc+(car_moving_history+i)->history[sim_step].link)->length){

                int now_pos = (car_moving_history+i)->history[sim_step].pos;
                int now_link = (car_moving_history+i)->history[sim_step].link;

                //check the next link
                int next_link = (car_moving_plan+i)->plan_arc[car_step[i]+1];


                (car_moving_history+i)->history[sim_step+1].pos -= (adr_node_arc.arc+now_link)->length;
                car_step[i]++;
                }
        }

        //履歴を表示  --  OK
         // for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
         //     if((car_moving_history+i)->arrived)
         //         break;
         // printf("Car %d in link %d pos %d at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,sim_step);
         // }
        sim_step++;
    }
        //特定車の履歴を表示
        int display_carID = 5;
       //  printf("start_node = %d end_node = %d \n",(_info_car_moving->car_moving_plan+display_carID)->plan[0],(_info_car_moving->car_moving_plan+display_carID)->plan[(_info_car_moving->car_moving_plan+display_carID)->plan_length-1]);
       // printf("Car %d have %d steps\n",display_carID,(_info_car_moving->car_moving_plan+display_carID)->plan_length);
       //  for(int i = 0 ; i <MAX_PERIOD_of_CAR_EXISTENCE;i++){
       //      printf("Car %d in link %d pos %d at %d steps\n",display_carID,(_info_car_moving->car_moving_history+display_carID)->history[i].link,(_info_car_moving->car_moving_history+display_carID)->history[i].pos,i);
       // }
    

    //check the num of car in every step --- OK
    // for(int loop_step = 0 ; loop_step < sim_step ; loop_step++){
    //   int cars_num = 0 ;
    //   for(int loop_arc = 0 ; loop_arc < adr_node_arc.num_arc ; loop_arc++)
    //     for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+loop_arc)->num_blocks ; loop_block++){
    //       cars_num += ((adr_node_arc.arc+loop_arc)->block[loop_block][loop_step].block_car_num);
    //     }
    //     printf("cars_num = %d at %d steps\n",cars_num,loop_step );
    //   }


     //watch the total_num in everystep --  OK
     // for(int k = 0 ; k < sim_step ; k++){
     //  int total_num = 0;
     // for(int i = 0 ; i < adr_node_arc.num_arc ; i++)
     //   for(int j = 0 ; j < (adr_node_arc.arc+i)->num_blocks ; j++){
     //    total_num += (adr_node_arc.arc+i)->block[j][0].block_car_num;
     //   }
     //   printf("total_num = %d in %d steps\n",total_num , k );
     // }
      

    //watch the Block[0] of Arc5 
    // for(int i = 0 ; i < sim_step ; i++)
    //   printf("Arc 5 block 0 have %d cars in %d steps\n", (adr_node_arc.arc+5)->block[0][i].block_car_num,i);
     printf("Done\n");
           
    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    //  printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
}


_INFO_CAR_MOVING *car_move_forever_RIS(struct _INFO_NODE_ARC adr_node_arc){
    //the memory of moving
    int memory = 0;
    //メモリを定義する
    car_moving_history = (CAR_MOVING_HISTORY*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    //check memory of car_memory_history
    if(car_moving_history){
     // printf("car_moving_history = %p\n",car_moving_history );
      printf("sizeof(car_moving_history) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY) );
      printf("checked memory of car_moving_history,sizeof(car_moving_history) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
    }
    else{
      printf("failed in check memory of car_moving_history\n");
      printf("End\n");
      exit(0);
    }
    car_moving_plan =(CAR_MOVING_PLAN*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    //check memory of car_moving_plan
    if(car_moving_plan){
     // printf("car_moving_plan = %p\n",car_moving_plan );
      printf("sizeof(car_moving_plan) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(car_moving_plan) is %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
    }
    else{
      printf("failed in check memory of car_moving_plan\n");
      printf("End\n");
      exit(0);
    }

    _info_car_moving = (_INFO_CAR_MOVING*)malloc(sizeof(_INFO_CAR_MOVING));
    //check memory of _info_car_moving
    if(_info_car_moving){
     // printf("car_moving_plan = %p\n",car_moving_plan );
     // printf("sizeof(_info_car_moving) = %d bytes\n",MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN) );
      printf("checked memory of car_moving_plan,sizeof(_info_car_moving) is %d bytes\n",sizeof(_INFO_CAR_MOVING));
    }
    else{
      printf("failed in check memory of _info_car_moving\n");
      printf("End\n");
      exit(0);
    }
    //printf("_info_car_moving = %p\n", _info_car_moving);


    _info_car_moving->num_car_moving = 0;
    _info_car_moving->car_moving_history = car_moving_history;
    _info_car_moving->car_moving_plan = car_moving_plan;

    //初期化
    int sim_step = 0;
        //CAR_MOVING_HISTORYを初期化   ---   OK
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
              car_step[i] = 0 ;
             (car_moving_history+i)->arrived = false;
             (car_moving_history+i)->car_ID = i ;
             (car_moving_history+i)->birth_step = 0; //-1 means not exist
            for(int j = 0 ; j < MAX_PERIOD_of_CAR_EXISTENCE;j++){
                (car_moving_history+i)->history[j].link  = -1;   //-1 means not exist
                (car_moving_history+i)->history[j].pos   = -1;
                (car_moving_history+i)->history[j].speed = 10;
            }
         }

       //CAR_MOVING_PLANを初期化    ---   OK
      for(int i = 0 ; i < MAX_CAR_NUM ; i++){
         (car_moving_plan+i)->plan_length = 0;
            for(int j = 0 ; j <MAX_PLAN_LENGTH;j++){
                (car_moving_plan+i)->plan[j] = -1;
            (car_moving_plan+i)->plan_arc[j] = -1;
            }
         }
    
    printf("車の情報初期化完了\n");

    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    //  printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
    printf("最大車輌数(%d)で実験する\n",MAX_CAR_NUM);
    _info_car_moving->num_car_moving = MAX_CAR_NUM;
    //main part of moving
    while(sim_step < MAX_SIM_STEP){

        //Init the plan of moving
              if(sim_step == 0){
                for(int i = 0 ; i < MAX_CAR_NUM;i++){
                    car_move_plan_allrand(adr_node_arc,*_info_car_moving,i,sim_step);
                }
              }

        //到着を判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if(car_step[i] >= ((car_moving_plan+i)->plan_length-1)){
                if((car_moving_history+i)->history[sim_step].link == -1){
                  // (car_moving_history+i)->arrived = true;
                    // int now_link = (car_moving_history+i)->history[sim_step-1].link;
                    // int loop_start_node = (adr_node_arc.arc+now_link)->des_node;
                   // car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,loop_start_node,sim_step);
                  //  (car_moving_history+i)->arrived = false;
                  int last_node = (adr_node_arc.arc+(car_moving_history+i)->history[sim_step-1].link)->des_node;
                  car_move_plan_endrand(adr_node_arc,*_info_car_moving,i,last_node,sim_step);
                    car_step[i]=0;
                   }
            }
        }

        //Dijkstraで計算した経路で移動する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
               (car_moving_history+i)->history[sim_step].link = (car_moving_plan+i)->plan_arc[car_step[i]];
               (car_moving_history+i)->history[sim_step+1].pos = (car_moving_history+i)->history[sim_step].pos+10 * (car_moving_history+i)->history[sim_step].speed;
               int now_link = (car_moving_history+i)->history[sim_step].link;
               int now_pos  = (car_moving_history+i)->history[sim_step].pos;
               //printf("car%d was in link %d pos %d ,at %d steps\n",i,now_link,now_pos,sim_step);
               double block_pos = (adr_node_arc.arc+now_link)->block[0][sim_step].block_length;
               
               //detect in whick block
                int loop_num_blocks = 0;
                while(loop_num_blocks < (adr_node_arc.arc+now_link)->num_blocks){                  
                  if(now_pos < block_pos)
                    break;
                  else
                      {
                       loop_num_blocks++;
                       block_pos +=(adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_length;
                      }
                }
                //show the result
                //printf("car %d in link %d block %d\n", i , now_link ,loop_num_blocks );
                //add the car to the block
                (adr_node_arc.arc+now_link)->block[loop_num_blocks][sim_step].block_car_num++;               
        }

        //次のアークに移動したを判断する
        for(int i = 0 ; i < MAX_CAR_NUM ; i++){
            if((car_moving_history+i)->history[sim_step+1].pos >= (adr_node_arc.arc+(car_moving_history+i)->history[sim_step].link)->length){
                int now_pos = (car_moving_history+i)->history[sim_step].pos;
                int now_link = (car_moving_history+i)->history[sim_step].link;


                (car_moving_history+i)->history[sim_step+1].pos -= (adr_node_arc.arc+now_link)->length;
                car_step[i]++;
                }
        }
        //履歴を表示  --  OK
         // for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
         //     if((car_moving_history+i)->arrived)
         //         break;
         // printf("Car %d in link %d pos %d at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,sim_step);
         // }
        sim_step++;

  //      printf("step = %d\n",sim_step);
    }
        //特定車の履歴を表示
        int display_carID = 5;
       //  printf("start_node = %d end_node = %d \n",(_info_car_moving->car_moving_plan+display_carID)->plan[0],(_info_car_moving->car_moving_plan+display_carID)->plan[(_info_car_moving->car_moving_plan+display_carID)->plan_length-1]);
       // printf("Car %d have %d steps\n",display_carID,(_info_car_moving->car_moving_plan+display_carID)->plan_length);
       //  for(int i = 0 ; i <MAX_PERIOD_of_CAR_EXISTENCE;i++){
       //      printf("Car %d in link %d pos %d at %d steps\n",display_carID,(_info_car_moving->car_moving_history+display_carID)->history[i].link,(_info_car_moving->car_moving_history+display_carID)->history[i].pos,i);
       // }
    

    //check the num of car in every step --- OK
    // for(int loop_step = 0 ; loop_step < sim_step ; loop_step++){
    //   int cars_num = 0 ;
    //   for(int loop_arc = 0 ; loop_arc < adr_node_arc.num_arc ; loop_arc++)
    //     for(int loop_block = 0 ; loop_block < (adr_node_arc.arc+loop_arc)->num_blocks ; loop_block++){
    //       cars_num += ((adr_node_arc.arc+loop_arc)->block[loop_block][loop_step].block_car_num);
    //     }
    //     printf("cars_num = %d at %d steps\n",cars_num,loop_step );
    //   }


     //watch the total_num in everystep --  OK
     // for(int k = 0 ; k < sim_step ; k++){
     //  int total_num = 0;
     // for(int i = 0 ; i < adr_node_arc.num_arc ; i++)
     //   for(int j = 0 ; j < (adr_node_arc.arc+i)->num_blocks ; j++){
     //    total_num += (adr_node_arc.arc+i)->block[j][0].block_car_num;
     //   }
     //   printf("total_num = %d in %d steps\n",total_num , k );
     // }
      

    //watch the Block[0] of Arc5 
    // for(int i = 0 ; i < sim_step ; i++)
    //   printf("Arc 5 block 0 have %d cars in %d steps\n", (adr_node_arc.arc+5)->block[0][i].block_car_num,i);
     printf("Done\n");
           
    // for(int i = 0 ; i < MAX_CAR_NUM ; i++)
    //  printf("CarID %d birth_step = %d \n",i,(car_moving_history+i)->birth_step);
}



#include <kilolib.h>

#define THRESH_LO 300
#define THRESH_HI 600

#define RB_SIZE 12// size of ring buffer
#define MAXN 10 //number of neighbors
#define GRADIENT_MAX 255

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define DESIRED_DISTANCE 45


//-----------Define general struct -------------//
typedef struct 
{
  uint32_t timestamp;
  uint8_t ID;
  uint8_t dist;
  uint8_t state;
  uint8_t t;
} 
Neighbor_t; // neighbor kilobot, that store id, distance and timestamp of neighbor

typedef struct {
    message_t msg;
    distance_measurement_t dist;
} 
received_message_t; // put message and distance of neighbor together

typedef struct{
  Neighbor_t neighbors[MAXN];
  uint8_t N_Neighbors;
  received_message_t RXBuffer[RB_SIZE];
  uint8_t RXHead;
  uint8_t RXTail;
  uint8_t id_of_nearsest_neighbor;
  uint8_t state_of_neighbor;
  uint8_t t;
  uint8_t timestamp; // type of struct 
}
neighbor_message_buffer; // the buffer which store the data of neighbor 
                         // ring buffer,store message in a ring queue(without overflow)

//------------------variable for motion and measuring light-------------//

uint8_t last_changed_motion = LEFT;
uint8_t current_motion = STOP;
int current_light = 0; 

message_t transmit_msg, msg;

neighbor_message_buffer swarm_neighbors; //---------kilobot nachbaren---
                                        // store the kilobot which have smaller id
neighbor_message_buffer obstacle;   // hinderniss
neighbor_message_buffer greater_id_neighbor; // store the kilobot which have greater id/
//--------------flag variable for moving--------------------------//

uint8_t direction = 100; // richtung fuer umfahren;  
uint8_t exit_flag, edge_flag,obstacle_flag ,stop_flag;// flag

//-----------variable for gradient set up--------------------//
uint8_t own_id;  // id of kilobot
uint8_t own_gradient = GRADIENT_MAX; // gradient value of kilobot
uint8_t received_gradient = GRADIENT_MAX; // received gradient id of kilobot
uint8_t gradient_flag = 0; // flag for gradient setup 

uint8_t lock = 0; // lock to compute moving
//-------------variable for distance and state of kilobot--------//
uint8_t distance_to_obstacle;   // nearest distance to obstacle
uint8_t distance_to_neighbor;   // nearest distance to greater id kilobot
uint8_t distance_to_greater_id_neighbor;  // nearest distance to smaller id kilobot
uint8_t state_of_neighbor = 0;  // flag(state ) of nearest neighbor kilobot

uint8_t distance_stop_station = 100; // distance to stop station

uint8_t other_motion; 

uint8_t old_id; // old id of nearst neighbor 


uint8_t count  = 0;
uint8_t t = 0;      
uint8_t k = 0;    

//-------------Define color-----------------//
uint8_t colorNum[] = {
  RGB(2,0,1),  //0 - magenta
  RGB(3,0,0),  //1 -**red
  RGB(3,1,0),  //2 - orange
  RGB(2,2,0),  //3 - yellow
  RGB(1,3,0),  //4 - yellowish green
  RGB(0,3,0),  //5 -**green
  RGB(0,2,2),  //6 - cyan
  RGB(1,0,2),  //7 -purple 
  RGB(0,0,3),  //8 -**blue
  RGB(0,1,2),  //9 - sky blue
  RGB(0,0,0)   //10- turn off
};
 
/////////////////////////////////////////////////////////
//-------------------define function--------------------//

void set_motion(int new_motion);// Function to handle motion.

void sample_light(); // Function to sample light.

//------------- Ring buffer data structur---------------// RB: ring buffer
uint8_t RB_empty(neighbor_message_buffer* buffer); 

uint8_t RB_full(neighbor_message_buffer* buffer);

received_message_t* RB_front(neighbor_message_buffer* buffer);

received_message_t* RB_back(neighbor_message_buffer* buffer);

void RB_popfront(neighbor_message_buffer* buffer);

void RB_pushback(neighbor_message_buffer* buffer);

//-------------------message processing----------------//
void process_message(neighbor_message_buffer* buffer); // process received message

void setup_message(); // set up transmited message 

void rxbuffer_push(neighbor_message_buffer* buffer, message_t *msg, distance_measurement_t *dist);/*
push received message in in ring buffer*/

//-------------------neigbor processing-------------//
void purgeNeighbors(neighbor_message_buffer* buffer); // remove the older neighbor 

void receive_inputs(neighbor_message_buffer* buffer);// message in ring buffer processing

uint8_t find_nearest_N_dist(neighbor_message_buffer* buffer); // compute the nearest neighbor

//----------------MOVEMENT and WAITING -----------------------//
void wait();

void follow_edge(neighbor_message_buffer* buffer, uint8_t dist);

void move_to_light(); // light verfolgung

//--------------- gradient value processing---------------------//
void gradient_setup(); // compare received gradient value

uint8_t received_gradient_value(message_t* msg, distance_measurement_t* dist);// compute received gradient value

//-----------------Flag of kilobot processing----------------------//
///////  (wichtigste funktion!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)

void set_flag(); // compute and compare distances to determine flag of kilobot

uint8_t flag_to_move(); // compute the behavior of kilobot 



void set_color_delay(uint8_t color); // set color of kilobot and blink

//----------------collision extension------------------------//
uint8_t state_of_neighbors(neighbor_message_buffer *buffer); // compute state of nearest kilobot

uint8_t find_dist_in_array_neighbor(neighbor_message_buffer * buffer, uint8_t ID);
// find distance of neighbor kilobot by ID
uint8_t find_neighbor_in_obstacle(neighbor_message_buffer* buffer); // find neighbor what is in the obstacle
///////////////MAIN FUNCTION//////////////////////////

void setup() {
    // put your setup code here, to be run only once
    exit_flag = 0;
    edge_flag = 0;
    stop_flag = 0;
    obstacle_flag = 0;
    swarm_neighbors.RXHead = 0;
    swarm_neighbors.RXTail = 0;
    swarm_neighbors.timestamp = 64;
    obstacle.RXHead = 0;
    obstacle.RXTail = 0;
    obstacle.timestamp = 64;
    greater_id_neighbor.RXHead = 0;
    greater_id_neighbor.RXTail = 0;
    greater_id_neighbor.timestamp = 64;

    if (kilo_uid < 10 && kilo_uid > 0){
      own_gradient = kilo_uid;
    }

    setup_message();
}

void loop() {
if(stop_flag == 2) {
  count = 0;
  wait();
  return;} 
  //---------exchange gradient message  in 20 second--------------//
  if(gradient_flag == 0){ // set up gradient, id
    gradient_setup();
    if(kilo_ticks > 640) gradient_flag = 1;   
    }
  //---------determine gradient value and set up color ----------//
  else if(gradient_flag == 1){
    own_id = own_gradient;
    if( own_id != 255)
      set_color(colorNum[own_id/10]);
    transmit_msg.data[1] = own_id;
    transmit_msg.crc = message_crc(&transmit_msg);
    msg.type = NORMAL;
    msg.data[0] = own_id + 9;
    msg.crc = message_crc(&msg);
    if(kilo_ticks > 960) {
      gradient_flag = 2;
    }
  }
  //------------MOVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE--------///
  else { 
    set_motion(STOP);

    receive_inputs(&swarm_neighbors);   
    receive_inputs(&obstacle);  
    receive_inputs(&greater_id_neighbor);

    distance_to_obstacle = find_nearest_N_dist(&obstacle);
    distance_to_neighbor = find_nearest_N_dist(&swarm_neighbors);
    distance_to_greater_id_neighbor = find_nearest_N_dist(&greater_id_neighbor);

    state_of_neighbors(&swarm_neighbors);
    

    set_flag();
    uint8_t move_flag = flag_to_move();
    lock = 0 ;
    
    if(lock == 0){
      if(distance_stop_station < 40 && stop_flag == 1){
        transmit_msg.data[0] = 111;
        transmit_msg.crc = message_crc(&transmit_msg);
        stop_flag = 2;
        wait();
        return;
      }
      if(edge_flag == 0 ){// kilobots move normal  
        if(move_flag == 4 ) { 
          if(distance_to_greater_id_neighbor < 40)
          { 
            set_color(colorNum[7]);
            set_motion(other_motion);
          }
          else {
            set_color(colorNum[9]);
            if(other_motion == LEFT)
              set_motion(RIGHT);
            else set_motion(LEFT);
    
            }
          }

        else if(move_flag == 1 ) move_to_light();
        else if (move_flag == 2 || move_flag == 3)
          set_motion(move_flag);
        else wait();
      }

      else if(edge_flag == 1 && exit_flag == 0){ // kilobots move around to obstacle
        if (move_flag == 1 && obstacle_flag == 0){
          follow_edge(&obstacle,DESIRED_DISTANCE);
        }
        else{
          wait();
        }
      }

      lock = 1;
    }
    //older_distance = distance_to_greater_id_neighbor;
  }
}


void message_rx(message_t *m, distance_measurement_t *d){
  if(gradient_flag == 0){
   received_gradient = received_gradient_value(m,d);
  }
  else{
    uint8_t dist = estimate_distance(d);
    if(m->data[0] == 100 && dist < 65 && dist != 0) {
      if(m->data[1] < own_id){
        rxbuffer_push(&swarm_neighbors,m,d);
      }
      else if (m->data[1] > own_id && dist < 65){
        rxbuffer_push(&greater_id_neighbor,m,d);
      }  
    }
    else if(m->data[0] == 111){
      if(distance_stop_station > 38)
      distance_stop_station = dist;
    }
    else if(m->data[0] != 100 ){
      if(edge_flag == 1)
        rxbuffer_push(&obstacle,m,d);
      else if(edge_flag == 0 && m->data[0] < 10)
        rxbuffer_push(&obstacle,m,d);
    }
    else return;
  }
}

message_t *message_tx(){
  if(obstacle_flag == 1 )
  {
    set_color_delay(0);
    return &msg;
  }
  else
    return &transmit_msg;
}
//------------------MAIN------------------------------//
int main() {
    // initialize hardware
    kilo_init();
    // start program
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

    kilo_start(setup, loop);

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////



void process_message(neighbor_message_buffer* buffer){
	uint8_t i;
	uint8_t ID;
  uint8_t state = 0 ;
  uint8_t t = 0;
	uint8_t *data = buffer->RXBuffer[buffer->RXHead].msg.data;
  
  if(data[0] == 100){
    ID = data[1];
    state = data[2];
    t = data[3];
  }
  else{
      ID = data[0];
  }

	uint8_t d = estimate_distance(&buffer->RXBuffer[buffer->RXHead].dist);
  if(d >= 70) d = 90;

	for(i = 0; i < buffer->N_Neighbors; i++)
	{
		if(buffer->neighbors[i].ID == ID)
			break;
	}
	if( i == buffer->N_Neighbors){
		if(buffer->N_Neighbors < MAXN -1)
			buffer->N_Neighbors++;
	}

	buffer->neighbors[i].ID = ID;
	buffer->neighbors[i].dist = d;
	buffer->neighbors[i].timestamp = kilo_ticks;
  buffer->neighbors[i].state = state; 
  buffer->neighbors[i].t = t;
}

void purgeNeighbors(neighbor_message_buffer* buffer){
	for (int i = buffer->N_Neighbors -1; i >= 0; i--)
	{
		if(kilo_ticks - buffer->neighbors[i].timestamp > buffer->timestamp){
			buffer->neighbors[i] = buffer->neighbors[buffer->N_Neighbors -1];
			buffer->N_Neighbors --;
		}
	}
}

void setup_message(){
	transmit_msg.type = NORMAL;
	transmit_msg.data[0] = 100;
  transmit_msg.data[1] = own_gradient;
  transmit_msg.data[2] = 0;
	transmit_msg.crc = message_crc(&transmit_msg);
}

void receive_inputs(neighbor_message_buffer* buffer){
	while(!RB_empty(buffer)){
		process_message(buffer);
		RB_popfront(buffer);
	}
	purgeNeighbors(buffer);
}

uint8_t find_nearest_N_dist(neighbor_message_buffer* buffer){
  uint8_t dist = 90;
  if(buffer->N_Neighbors == 0)
    return dist;
  buffer->id_of_nearsest_neighbor = 100;
  buffer->state_of_neighbor = 0;
  buffer->t = 0;
  for (int i = 0; i < buffer->N_Neighbors; ++i)
  {
  	if(dist > buffer->neighbors[i].dist){
  		dist = buffer->neighbors[i].dist;
  		buffer->id_of_nearsest_neighbor = buffer->neighbors[i].ID;
  	}
  }
  return dist;
}

void follow_edge(neighbor_message_buffer* buffer, uint8_t dist){	
	if(direction % 2 == 1 ){
		if(distance_to_obstacle < dist)
      set_motion(LEFT);     
		else
      set_motion(RIGHT);
	}
	else if(direction % 2 == 0 ){
		if(distance_to_obstacle < dist)   
			set_motion(RIGHT);    
		else 
      set_motion(LEFT);
    
	}
}


void wait(){
  if(current_motion != STOP)
    set_motion(STOP);
  set_color_delay(1);
 }


uint8_t RB_empty(neighbor_message_buffer* buffer){
  return buffer->RXHead == buffer->RXTail;
}

uint8_t RB_full(neighbor_message_buffer* buffer){
  return (buffer->RXHead +1)%RB_SIZE == buffer->RXTail;
}

received_message_t* RB_front(neighbor_message_buffer* buffer){
  return &buffer->RXBuffer[buffer->RXHead];
}

received_message_t* RB_back(neighbor_message_buffer* buffer){
  return &buffer->RXBuffer[buffer->RXTail];
}

void RB_popfront(neighbor_message_buffer* buffer){
  buffer->RXHead = (buffer->RXHead + 1)%RB_SIZE;
}

void RB_pushback(neighbor_message_buffer* buffer){
    buffer->RXTail = (buffer->RXTail+1)%RB_SIZE;
  if(RB_empty(buffer)){
    buffer->RXHead = (buffer->RXHead + 1)%RB_SIZE;
  }
}

void rxbuffer_push(neighbor_message_buffer* buffer, message_t *msg, distance_measurement_t *dist){
  received_message_t* rmsg = RB_back(buffer);
  rmsg->msg = *msg;
  rmsg->dist = *dist;
  RB_pushback(buffer); 
}

void gradient_setup(){
  if (own_gradient == 255 && own_gradient > received_gradient + 10){
      own_gradient = received_gradient + 10;

      // Update the transmission message whenever the gradient changes.
      transmit_msg.type = NORMAL;
      transmit_msg.data[1] = own_gradient;
      transmit_msg.crc = message_crc(&transmit_msg);
  }
}

uint8_t received_gradient_value(message_t* msg, distance_measurement_t* dist){
	uint8_t distance = estimate_distance(dist);
    if(msg->data[0] == 100 && distance < 45){
        if(kilo_uid > 0 && kilo_uid < 10){
          set_color_delay(1);
        }
        else{
         set_color_delay(7);
    	 }
        return  msg->data[1];
    }
    else return GRADIENT_MAX;
}

void set_color_delay(uint8_t color){
	set_color(colorNum[color%10]);
	delay(200);
	set_color(RGB(0,0,0));
}

uint8_t flag_to_move(){

    // 0 :wait, 1 : move to light or follow edge; 2 and 3: turn left and turn right, 4 drive around 

  if(edge_flag == 0 ){
    
    if(distance_to_greater_id_neighbor > 40)
      count = 0;  

    if(stop_flag == 1) return 1;


    if(find_neighbor_in_obstacle(&greater_id_neighbor) == 1){
        count = 0;
        return 0; 
      }

    else if(distance_to_neighbor < 60 && state_of_neighbor == 0 && stop_flag == 0){
      count = 0;
      return 0;
    }
    
    else if(distance_to_neighbor < 65 && state_of_neighbor != 0 && stop_flag == 0){
      if(t == 0)
        return 0;
      else if(t == 1 && distance_to_neighbor < 50){
        set_color(colorNum[3]);
        return state_of_neighbor;
      }
      else return 1;
    }

    else if(greater_id_neighbor.t == 1 && distance_to_greater_id_neighbor < 50){
        return greater_id_neighbor.state_of_neighbor;
    }
    
    else if(distance_to_greater_id_neighbor < 40 ){
      count += 1;
      if(count == 1) old_id = greater_id_neighbor.id_of_nearsest_neighbor;
      if(count <= 6){
        return 1;
      }
      else {
        if(old_id  == greater_id_neighbor.id_of_nearsest_neighbor){
          other_motion = last_changed_motion ;
          return 4;
        }   
        else {
          count = 0;
          return 1;
        }
      }
    }
    else return 1;

  }
  else { // edge_flag = 1
    if(distance_to_obstacle < DESIRED_DISTANCE){
      if(distance_to_neighbor < 45 && state_of_neighbor != 0){
        obstacle_flag = 1;
        return 0;
      }
      //else if(distance_to_neighbor < 50 && state_of_neighbor == 0)
        //return 0; 
      else if(distance_to_greater_id_neighbor < 45 || distance_to_neighbor < 45){
        if(k == 1){
          return 0;
        }
        else return 1; 
      }
      else return 1;
    }
    else return 1;
  }
 
}


void set_flag(){
	if(distance_to_obstacle < 50 && exit_flag == 0 && edge_flag == 0){
		edge_flag = 1;
    set_color_delay(8);
		direction = obstacle.id_of_nearsest_neighbor;
    if(direction % 2 == 1){ 
      transmit_msg.data[2] = LEFT; 
    }
    else{
      transmit_msg.data[2] = RIGHT;
    }
        
    //transmit_msg.data[3] = k;
    transmit_msg.crc = message_crc(&transmit_msg);
    return;
	}
  if(edge_flag == 1){
    if(obstacle_flag == 1){
      if(distance_to_neighbor < 60){
        obstacle_flag = 0;
        //set_color_delay(8);
        return;
      }
      if(distance_to_obstacle > 65)
        edge_flag = 0;
    }
    else if(find_dist_in_array_neighbor(&obstacle,4) < 50 || find_dist_in_array_neighbor(&obstacle,5) < 50){      
      k = 1;
      transmit_msg.data[3] = k;
      transmit_msg.crc = message_crc(&transmit_msg);
      set_color(RGB(1,1,1));
    }
	  else if((obstacle.id_of_nearsest_neighbor == 8 || obstacle.id_of_nearsest_neighbor == 9) && distance_to_obstacle < 55){
		  edge_flag = 0;
		  exit_flag = 1;
      transmit_msg.data[2] = 0;
      transmit_msg.data[3] = 0;
      transmit_msg.crc = message_crc(&transmit_msg);
      set_color(colorNum[1]);
      return;
      }
    }
    if(distance_stop_station < 70 && stop_flag == 0)
      stop_flag = 1;
}


void set_motion(int new_motion)
{
    // Only take an an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {
            set_motors(0, 0);
            delay(500);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
            //delay(500);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            last_changed_motion = LEFT;
            set_motors(kilo_turn_left, 0);
            delay(500);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            last_changed_motion = RIGHT;
            set_motors(0, kilo_turn_right);
            delay(500);
        }
    }
}

// Function to sample light.
 void sample_light()
 {
     // The ambient light sensor gives noisy readings. To mitigate this,
     // we take the average of 300 samples in quick succession.

     int number_of_samples = 0;
     uint32_t sum = 0;


     while (number_of_samples < 300)
     {
         int sample = get_ambientlight();

         // -1 indicates a failed sample, which should be discarded.
         if (sample != -1)
         {
             sum = sum + sample;
             number_of_samples = number_of_samples + 1;
         }
     }
     // Compute the average.
     current_light = sum / number_of_samples;
     delay(10);
  }

void move_to_light(){
  set_color(colorNum[8]);
    set_motion(last_changed_motion);
    sample_light();
     if (current_light < THRESH_LO)
     {
         set_motion(RIGHT);
     }
     else if (current_light > THRESH_HI)
     {
         set_motion(LEFT);
     }
 }

uint8_t state_of_neighbors(neighbor_message_buffer *buffer){
  uint8_t dist = 90;
  if(buffer->N_Neighbors == 0)
    return dist;
  state_of_neighbor = 0;
  t = 0;
  for (int i = 0; i < buffer->N_Neighbors; ++i)
  {
    if(dist > buffer->neighbors[i].dist){
      state_of_neighbor = buffer->neighbors[i].state;
      t = buffer->neighbors[i].t;
    }
  }
  return dist;
 }

uint8_t find_dist_in_array_neighbor(neighbor_message_buffer * buffer, uint8_t ID){
  uint8_t dist = 90;
  for (int i = 0; i < buffer->N_Neighbors; ++i){
    if(buffer->neighbors[i].ID == ID){
      dist = buffer->neighbors[i].dist;
      break;
    }
  }
  return dist;
 }

uint8_t find_neighbor_in_obstacle(neighbor_message_buffer* buffer){
  for (int i = 0; i < buffer->N_Neighbors; i++){
    if(buffer->neighbors[i].state != 0 && buffer->neighbors[i].dist < 60 && buffer->neighbors[i].t == 0)
      return 1;
  }
  return 0; 
}


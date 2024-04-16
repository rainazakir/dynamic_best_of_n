/*-----------------------------------------------------------------------------------------------*/
/* This file provides functions need to be implemented for Kilobot in order to work on the       */
/*  Kilogrid to test symmetry breaking two options with self-sourced noise and social interaction*/
/*-----------------------------------------------------------------------------------------------*/

// macro if we are in sim or reality -> command out if on real robot
#define SIMULATION


/*-----------------------------------------------------------------------------------------------*/
/* Imports - depending on the platform one has different imports                                 */
/*-----------------------------------------------------------------------------------------------*/

#include "kilolib.h"
#include <kilolib.h>
#include <math.h>
#include <time.h>    // time()
#ifdef SIMULATION

#include <stdio.h>
#include <float.h>
#include "agent.h"
#include <debug.h>


#else

#include "utils.h"  // TODO check if this is needed ?!?
#include "kilob_tracking.h"
#include "kilo_rand_lib.h"
#include "../communication.h"
#include "kilob_messaging.h"


#endif


/*-----------------------------------------------------------------------------------------------*/
/* Define section here you can define values, e.g., messages types                               */
/*-----------------------------------------------------------------------------------------------*/
// options
#define UNCOMMITTED 99  //opinion for uncommitted
// message types
#define FROMBOT 9  //the message is from robot and not kilogrid
#define INIT_MSG 10  // initial message from the kilogrid
#define GRID_MSG 11  // info msg from the kilogrid with option and position
#define VIRTUAL_AGENT_MSG 12  // msg forwarded from the kilogrid
#define TO_KILOGRID_MSG 62
#define min(a,b) ((a) < (b) ? a : b)
#define INFOPTION 2
#define SWARMSIZE 100
//#define AGENTS_PER_GROUP 20
//#define GROUPS 5
#define MSG_KILOG 1
#define MSG_NEIGHBOUR 1

#define kval 0 //0 all majority

#define OMG 0.25
#define ALPHA 0.2
/*-----------------------------------------------------------------------------------------------*/
/* Change these when running experiment                                                          */
/*-----------------------------------------------------------------------------------------------*/
//double noise = 0.99; // SET THIS TO -1 FOR NO NOISE, 0.1--> 0.05, 0.5-->0.25
/*-----------------------------------------------------------------------------------------------*/

//opinion = A -->1   //opinion = B --> 2  //uncommited = C --> UNCOMITTED
int currentopinion; //1

double timer; // to hold time to be in each state
double avg_exploration_time = 800.0; //***--> time to be in exploration state--> fixed
//double avg_uncommitted_time = 200.0; // time to stay in dissemination for uncommitted agents
///double dissemparam = 1300.0;

int foundmodules[18][38] = {0}; //to keep track of tiles visited in one exploration cycle


int estimateR;
int estimateB;
int estimateG;
int estimateY;
int estimateBL;
int estimateN;

int avg_neighbours;


int tot_e;

//int state = 0; //0--> Exploration , 1-->Dissemination , 2-->Voting // start in exploration state

/*-----------------------------------------------------------------------------------------------*/
/* Enum section - here we can define useful enums                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef enum{
    false = 0,
    true = 1,
} bool;

/* Enum for different motion types */
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

/* Enum for different states */
typedef enum {
    EXPLORATION,
    FILTER,
} state;

/* Enum for different wall function  states */
typedef enum {
    TURN_TO_AVOID_WALL,
    STRAIGHT_TO_AVOID_WALL,
    COMPLETE_WALL_AVOIDANCE,
} wall_state;
/*-----------------------------------------------------------------------------------------------*/
state current_state = EXPLORATION;
wall_state wall_function_state = TURN_TO_AVOID_WALL;
/*-----------------------------------------------------------------------------------------------*/
/* Motion related Variables                                                                      */
/*-----------------------------------------------------------------------------------------------*/
motion_t current_motion_type = STOP;
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; //*** constant to set maximum rotation to turn during random walk
const uint32_t max_straight_ticks = 300; //*** set the time to walk straight before randomly turning
const uint8_t max_wall_avoidance_turning_ticks = 130;
const uint32_t max_wall_avoidance_straight_ticks = 260;
//const uint32_t broadcast_ticks = 32;
uint32_t last_motion_ticks = 0;
uint32_t last_motion_wall_ticks = 0;

uint8_t kilogrid_commitment = 0;  // This is the initial commitment attained from kilogrid
//float my_commitment_quality = 0.0;
int last_changed = 0;
bool wall_avoidance_turning = false;
bool wall_avoidance_straight = false;

int model = 1;
//uint8_t communication_range = 0;  // communication range in cells

/*-----------------------------------------------------------------------------------------------*/
/* Communication variables - used for communication and stuff                                    */
/*-----------------------------------------------------------------------------------------------*/
bool broadcast_msg = false;
// how often we try to send the msg - in simulation once is sufficient
#ifdef SIMULATION
#define MSG_SEND_TRIES 1
#else
#define MSG_SEND_TRIES 10
#endif
// Kilobot -> Kilogrid
uint32_t msg_counter_sent = MSG_SEND_TRIES + 1;  // counts the messages sent
uint32_t msg_number_send = 0;  // change if you want to send a msg
uint32_t msg_number_current_send = 0;  // var for checking against the last
// Kilogrid -> Kilobot
bool init_flag = false;
bool received_grid_msg_flag = false;
bool received_virtual_agent_msg_flag = false;

int op_message_env_voter = 0;
int op_message_neighbour_voter = 0;


// message content
#ifdef SIMULATION

//uint8_t communication_range_msg = 0;
//uint8_t x_pos_msg = 0;
//uint8_t y_pos_msg = 0;
//uint32_t msg_counter = 0;

#else
IR_message_t* message;
#endif

// Flag to keep track of new messages.
message_t message;
int new_message = 0;

int last_changed;
// Flag to keep track of message transmission.
int message_sent = 0;

int storeid[SWARMSIZE] = {};

int received_option; //to save an opinion received from another bot
int received_option_kilogrid; // to save option received from kilogrid
int received_uid; //previously used to save neighbours kilouid temporarily
int wall_flag; //to check if wall signal or not
/*-----------------------------------------------------------------------------------------------*/
/* Arena variables                                                                               */
/*-----------------------------------------------------------------------------------------------*/

bool hit_wall = false;  // set to true if wall detected
bool wall_avoidance_state = false;
//to keep track of tiles and quality
int total_tiles_found;
int tiles_of_1_option;
int tiles_of_2_option;
int tiles_of_3_option;
int tiles_of_4_option;
int tiles_of_2_option;
int tiles_of_5_option;


int op_count[5] = {0, 0, 0, 0, 0};
int op_count_vm[5] = {0, 0, 0, 0, 0};

///NOT USED ANYMORE
//int neighbourid[SWARMSIZE] = {};//***change the size if running with more than 25 kilobots--> mention no of robots used in size
//int neighbouropinion[SWARMSIZE] = {}; //right now voter *** k->5, change to vary according to the majority rule

//sets the qratios, 6/3=2, change values to vary//  double q3 = 0.003;//***quality for A,// double q1 = 0.006;//*** quality for B
//double q3 = 0.003;//***qualities are same for A and B for now
//double q1 = 0.003;//***qualities are same for A and B for now

/*-----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------------*/
/* Setting the Motion of the Bot                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    if( current_motion_type != new_motion_type ){

        switch( new_motion_type ) {
            case FORWARD:
                spinup_motors();
                set_motors(kilo_straight_left,kilo_straight_right);
                break;
            case TURN_LEFT:
                spinup_motors();
                set_motors(kilo_turn_left,0);
                break;
            case TURN_RIGHT:
                spinup_motors();
                set_motors(0,kilo_turn_right);
                break;
            case STOP:
                set_motors(0,0);
                break;
        }
        current_motion_type = new_motion_type;
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Random Walk                                                                                   */
/*-----------------------------------------------------------------------------------------------*/
void random_walk(){
    switch( current_motion_type ) {
        case TURN_LEFT:
        case TURN_RIGHT:
            if(  kilo_ticks > last_motion_ticks + turning_ticks ) {
                /* start moving forward */
                last_motion_ticks = kilo_ticks;
                set_motion(FORWARD);
            }
            break;
        case FORWARD:
            //spinup_motors();
            //set_motors(20,20);
            if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
                /* perform a random turn */
                last_motion_ticks = kilo_ticks;
                if( rand()%2 ) {
                    set_motion(TURN_LEFT);
                    current_motion_type = TURN_LEFT;
                }
                else {
                    set_motion(TURN_RIGHT);
                    current_motion_type = TURN_RIGHT;
                }
                turning_ticks = rand()%max_turning_ticks + 1;
            }
            break;
        case STOP:
            set_motion(STOP);
        default:
            set_motion(FORWARD);
    }
}
/*-----------------------------------------------------------------------------------------------*/
/* Function to check if the robot is against the wall                                             */
/*-----------------------------------------------------------------------------------------------*/
/*void check_if_against_a_wall() {
    // when the hitwall flag is true -- (either at the border tiles or white buffer)
    if(hit_wall){
        if( rand()%2 ) {
            set_motion(TURN_LEFT);
            current_motion_type = TURN_LEFT;
            set_color(RGB(2, 0, 2));

            delay(5100);
            set_color(RGB(0, 0, 0));

            set_motion(FORWARD);
            delay(9000);

            random_walk();
        }else {
            set_motion(TURN_RIGHT);
            current_motion_type = TURN_RIGHT;

            set_color(RGB(2, 0, 2));
            delay(5100);
            set_color(RGB(0, 0, 0));
            set_motion(FORWARD);
            delay(9000);
            random_walk();
        }
        turning_ticks = rand()%max_turning_ticks + 1;

    }
}*/

void wall_avoidance_function(){
    wall_avoidance_state = true;
    hit_wall = false;

    if (wall_function_state == TURN_TO_AVOID_WALL){
        set_color(RGB(3, 0, 3));
        //printf("comes to turnning \n");
        if( rand()%2 ) {
            // while ((kilo_ticks - last_motion_ticks) < max_wall_avoidance_turning_ticks) {
            /* perform a random turn */
            set_motion(TURN_LEFT);
            current_motion_type = TURN_LEFT;
            // }
        }else {
            // while ((kilo_ticks - last_motion_ticks) < max_wall_avoidance_turning_ticks) {
            /* perform a random turn */
            set_motion(TURN_RIGHT);
            current_motion_type = TURN_RIGHT;
            //  }
        }
        last_motion_wall_ticks = kilo_ticks;
        wall_function_state = STRAIGHT_TO_AVOID_WALL;
    }
    if (wall_function_state == STRAIGHT_TO_AVOID_WALL) {
        //printf("comes to straight \n");

        if ((kilo_ticks - last_motion_wall_ticks) > max_wall_avoidance_turning_ticks) {
            /* start moving forward */
            set_color(RGB(0, 3, 0));
            last_motion_wall_ticks = kilo_ticks;
            set_motion(FORWARD);
            wall_function_state = COMPLETE_WALL_AVOIDANCE;

        }
    }
    if (wall_function_state == COMPLETE_WALL_AVOIDANCE) {
        //printf("comes to finish avoidance \n");

        if ((kilo_ticks - last_motion_wall_ticks) > max_wall_avoidance_straight_ticks) {
            last_motion_wall_ticks = kilo_ticks;
            wall_function_state = TURN_TO_AVOID_WALL;
            wall_avoidance_state = false;
            set_color(RGB(0, 0, 0));
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to get Exponential Distribution for timing to stay is Dissem state                   */
/*-----------------------------------------------------------------------------------------------*/
double ran_expo(double lambda){
    double u;
    u = rand() / (RAND_MAX + 1.0);
    return -log(1- u) / lambda;
}

/*-----------------------------------------------------------------------------------------------*/
/* Function to get a random 1 or 0 to check for self-sourced noise or social interaction         */
/*-----------------------------------------------------------------------------------------------*/
double r2()
{
    return (double)rand() / (double)RAND_MAX ;
}



int isNumberNotInArray(int number, int arr[], int size) {
    for (int i = 0; i < size; i++) {
        if (arr[i] == number) {
            return 0; // Number is found in the array
        }
    }
    return 1; // Number is not found in the array
}

/*-----------------------------------------------------------------------------------------------*/
/*                              The Exploration function                                         */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/
void gotoexploration(){

    //  random_walk(); //start with random walk

    //set led colours
    if (currentopinion == 1){
        set_color(RGB(3, 0, 0));

    } else if (currentopinion == 2){
        set_color(RGB(0, 0, 3));

    }else if (currentopinion == 3){
        set_color(RGB(0, 3, 0));

    }else if (currentopinion == 4){
        set_color(RGB(3, 3, 0));

    }else if (currentopinion == 5){
        set_color(RGB(3, 3, 3));

    }


    //if time for exploration not over yet, do nothing else move on to dissemination state
    if ((kilo_ticks - last_changed) < timer) {//check if still within time for exploration state or not
        //check_if_against_a_wall(); //check if hitting the wall
        double u = r2();
        if(u<0.5){
            broadcast_msg = true; //then send out message to other bots

        }else{
            broadcast_msg = false; //then send out message to other bots

        }
        if (new_message == 1) {

            if (isNumberNotInArray(received_uid, storeid, (sizeof(storeid) / sizeof(storeid[0])))) {

                //not cpunted
                if(model == 0){
                    op_count[received_option - 1] += MSG_NEIGHBOUR; //for majority rule
                    //op_count_message[received_option - 1] += MSG_NEIGHBOUR; //for majority rule
                    avg_neighbours +=1;

                }else {
                    op_message_neighbour_voter = received_option; // for voter model
                }
                //TODO
                // Calculate size of the array
                int size;
                for (size = 0; size < SWARMSIZE; size++) {
                    if (storeid[size] == 0) {
                        break;
                    }
                }
                storeid[size++] = received_uid;



                //check if comes here
            }


            //printf("Updated array: ");

            new_message = 0;
        }


    } else{ //if not in exploration state

        if(kilo_uid ==18){
        //  printf("%d tile my op 1, %d tile my op 2, ,%d tile my op 3, ,%d tile my op 4 ,%d tile my op 5,  %d total tiles \n", tiles_of_1_option,tiles_of_2_option,tiles_of_3_option,tiles_of_4_option,tiles_of_5_option,total_tiles_found);

            printf("Array before tiles: ");
            for (int i = 0; i < 5; i++) {
               printf("%d ", op_count[i]);
            }
            printf("\n ");
            printf("tot %d  and omg %f \n", (op_count[0]+ op_count[1]+ op_count[2]+ op_count[3]+ op_count[4]),OMG*(op_count[0]+ op_count[1]+ op_count[2]+ op_count[3]+ op_count[4]) );

            printf("%d tile my op 1, %d tile my op 2, ,%d tile my op 3, ,%d tile my op 4 ,%d tile my op 5,  %d total tiles \n", tiles_of_1_option,tiles_of_2_option,tiles_of_3_option,tiles_of_4_option,tiles_of_5_option,total_tiles_found);
           // int array_size = sizeof(storeid) / sizeof(storeid[0]);


            // Find the maximum value in the array
            //for (int i = 0; i < array_size; i++) {
              //  printf("store id %d ", storeid[i]);
              //  printf("\n ");
           // }
        }

        //
        //


        float x = OMG*(op_count[0]+ op_count[1]+ op_count[2]+ op_count[3]+ op_count[4]);
        //float x = OMG;
        //float x = (OMG/4.7)*(op_count[0]+ op_count[1]+ op_count[2]+ op_count[3]+ op_count[4]);
         x = roundf(x);
        if(kilo_uid ==18) {
            printf("x is %f \n", x);
            //x = roundf(x);
            //printf("x %f:\n ", x);
        }
        // Store the names of the variables in an array of strings
        //const char *variable_names[] = {"tiles_of_1_option", "tiles_of_2_option", "tiles_of_3_option", "tiles_of_4_option", "tiles_of_5_option"};
        int values[] = {tiles_of_1_option, tiles_of_2_option, tiles_of_3_option, tiles_of_4_option, tiles_of_5_option};
        int num_variables = sizeof(values) / sizeof(values[0]);

        // Find the maximum value
        int max_value = values[0];
        for (int i = 1; i < num_variables; ++i) {
            if (values[i] > max_value) {
                max_value = values[i];
            }
        }

        // Count the number of variables with the maximum value
        int count = 0;
        int max_indexes[num_variables];
        for (int i = 0; i < num_variables; ++i) {
            if (values[i] == max_value) {
                max_indexes[count++] = i;
            }
        }

        // If there are multiple variables with the maximum value, choose one randomly
        srand(time(NULL));
        int chosen_index;
        if (count > 1) {
            chosen_index = max_indexes[rand() % count];
        } else {
            chosen_index = max_indexes[0];
        }

        if(kilo_uid ==18){

            printf("chosen index %d:\n ", chosen_index);



        }

        op_count[chosen_index] += x;
        // Print the variable with the highest value
        //if (chosen_index+1 == 1) {
         //   op_count[0]+=tiles_of_1_option + x;

      //  } else if (chosen_index+1 == 2) {
        //    op_count[1]+=tiles_of_2_option+x;

       // } else if (chosen_index+1 == 3) {

        //    op_count[2]+=tiles_of_3_option+x;

       // } else if (chosen_index+1 == 4) {

         //   op_count[3]+=tiles_of_4_option+x;
      //  } else {

          //  op_count[4]+=tiles_of_5_option+x;
        //}

        op_count_vm[0]+=tiles_of_1_option;
        op_count_vm[1]+=tiles_of_2_option;
        op_count_vm[2]+=tiles_of_3_option;
        op_count_vm[3]+=tiles_of_4_option;
        op_count_vm[4]+=tiles_of_5_option;

        estimateR = tiles_of_1_option;
        estimateB = tiles_of_2_option;
        estimateG = tiles_of_3_option;
        estimateY = tiles_of_4_option;
        estimateBL = tiles_of_5_option;
        estimateN = avg_neighbours;

        //estimateR = op_count[0];
        //estimateB = op_count[1];
        //estimateG = op_count[2];
        //estimateY = op_count[3];
        //estimateBL = op_count[4];
        //estimateN = avg_neighbours;

        tot_e = estimateR+estimateB+estimateG+estimateY+estimateBL;
        if(kilo_uid ==18){

            printf("AFTER array TILES Array:\n ");
            for (int i = 0; i < 5; i++) {
                printf("%d ", op_count[i]);
                printf("\n ");

            }

           // printf("STORE ID Array: ");
           // for (int i = 0; i < SWARMSIZE; i++) {
          //      printf("%d ", storeid[i]);
          //  }
          //  printf("\n ");

        }


        current_state = FILTER;//go to Dissemination mode
            // set_color(RGB(0, 0, 0));


        last_changed = kilo_ticks;

        //reset the variable that are used to find the qr for next exploration-dissem cycle
       memset(foundmodules, 0, sizeof(foundmodules[0][0]) * 18 * 38);
        tiles_of_1_option = 0;
        tiles_of_2_option = 0;
        tiles_of_3_option = 0;
        tiles_of_4_option = 0;
        tiles_of_5_option = 0;
        total_tiles_found = 0;
         //Reset the array to empty
        for (int i = 0; i < SWARMSIZE; i++) {
            storeid[i] = 0;
        }

        //qratio = 0;
        // set_color(RGB(0, 0, 0));

    }

}


/*-----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------*/
/* NOT USED:Function to process the data received from the kilogrid regarding the environment    */
/*-----------------------------------------------------------------------------------------------*/
/*
void update_grid_msg() {
    // TODO add logic here
   // set_color(RGB(2, 2, 0));
  //  delay(2000);
    set_color(RGB(0, 0, 0));

    return;
}
*/
/*-----------------------------------------------------------------------------------------------*/
/*                          The Polling Function- Social interaction                             */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/
void poll(){


    if (model == 0) {

        int array_size = sizeof(op_count) / sizeof(op_count[0]);
        int max_sum_index = 0;
        int all_same_or_zero = 1; // Flag to check if all numbers are same or zero
        int max_count = 0; // Maximum value found so far

        // Find the maximum value in the array
        for (int i = 0; i < array_size; i++) {
            if (op_count[i] > max_count) {
                max_count = op_count[i];
            }
        }

        // Check if all numbers are the same or zero
        for (int i = 1; i < array_size; i++) {
            if (op_count[i] != op_count[0]) {
                all_same_or_zero = 0;
                break;
            }
        }

        // If all numbers are the same or zero, choose one index randomly
        if (all_same_or_zero) {
            srand(time(NULL)); // Seed the random number generator
            max_sum_index = rand() % array_size; // Choose a random index
        } else {
            // Find the indexes with the highest value
            int indexes[array_size];
            int count = 0;
            for (int i = 0; i < array_size; i++) {
                if (op_count[i] == max_count) {
                    indexes[count++] = i;
                }
            }
            srand(time(NULL)); // Seed the random number generator
            max_sum_index = indexes[rand() % count]; // Choose one of the indexes randomly
        }



        currentopinion= max_sum_index + 1;
        //printf("THE UPDATED OPINION I  %d \n ", currentopinion);





    }else { //for voter rule
        int array_size = sizeof(op_count_vm) / sizeof(op_count_vm[0]);
        int max_sum_index = 0;
        int all_same_or_zero = 1; // Flag to check if all numbers are same or zero
        int max_count = 0; // Maximum value found so far

        // Find the maximum value in the array
        for (int i = 0; i < array_size; i++) {
            if (op_count_vm[i] > max_count) {
                max_count = op_count_vm[i];
            }
        }

        // Check if all numbers are the same or zero
        for (int i = 1; i < array_size; i++) {
            if (op_count_vm[i] != op_count_vm[0]) {
                all_same_or_zero = 0;
                break;
            }
        }

        // If all numbers are the same or zero, choose one index randomly
        if (all_same_or_zero) {
            srand(time(NULL)); // Seed the random number generator
            max_sum_index = rand() % array_size; // Choose a random index
        } else {
            // Find the indexes with the highest value
            int indexes[array_size];
            int count = 0;
            for (int i = 0; i < array_size; i++) {
                if (op_count_vm[i] == max_count) {
                    indexes[count++] = i;
                }
            }
            srand(time(NULL)); // Seed the random number generator
            max_sum_index = indexes[rand() % count]; // Choose one of the indexes randomly
        }

        op_message_env_voter = max_sum_index + 1;
        // Generate a random number (0 or 1)
        //int random_choice = rand() % 2;
        double u = r2();

        // Choose randomly between values of two variables
        if (u <= ALPHA) {
            currentopinion = op_message_env_voter;

        } else {
            currentopinion = op_message_neighbour_voter;
        }
    }


    // if(kilo_uid ==12){


     //   printf("The chosen opinion is %d \n ", currentopinion);

    //}


    message.data[1] = currentopinion;
    message.data[2] = kilo_uid;
    //message.data[3]= estimateR;
    //message.data[4] = estimateB;
    //message.data[5]=estimateG;
    //
    // message.data[6]= tot_e;
    message.crc = message_crc(&message);


    //reset count array to empty
    for (int i = 0; i < 5; i++) {
        op_count[i] = 0;
    }

    //reset count array to empty
    for (int i = 0; i < 5; i++) {
        op_count_vm[i] = 0;
    }
    //go to exploration state
    current_state = EXPLORATION;
    //timer =  ran_expo(1.0/avg_exploration_time); // get the time for exploration
    timer = avg_exploration_time;
    last_changed = kilo_ticks;
    set_color(RGB(0, 0, 0));
    //reset the variable that are used to find the qr for next exploration-dissem cycle
    memset(foundmodules, 0, sizeof(foundmodules[0][0]) * 18 * 38);
    tiles_of_1_option = 0;
    tiles_of_2_option = 0;
    tiles_of_3_option = 0;
    tiles_of_4_option = 0;
    tiles_of_5_option = 0;
    total_tiles_found = 0;
    avg_neighbours = 0;

    // Reset the array to empty
    for (int i = 0; i < SWARMSIZE; i++) {
        storeid[i] = 0;
    }

}



/*-----------------------------------------------------------------------------------------------*/
/* NOT USED: Function to process the data received from the kilogrid regarding other robots      */
/*-----------------------------------------------------------------------------------------------*/
/*
void update_virtual_agent_msg() {
    // TODO add logic here
    return;
}

*/


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the callback, for when the robot receives an infrared message (here  */
/* also from the kilogrid and from other robots)                                                 */
/*-----------------------------------------------------------------------------------------------*/
// because there has been an "updated" version of the kilo_lib we have a slightly different
// implementation
#ifdef SIMULATION
void message_rx( message_t *msg, distance_measurement_t *d ) {
#else
    void message_rx( IR_message_t *msg, distance_measurement_t *d ) {
#endif
    // check the messages

    if(msg->type == FROMBOT){ //if message from another bot

        // printf("hey i am a bot");

        new_message = 1;        // Set the flag on message reception.
        received_option = msg->data[1]; //get its option
        received_uid = msg->data[2]; //get its uid
    }
    if(msg->type == GRID_MSG){ //if message from Kiogrid
        //printf("message from grid\n");
        //printf("%hhu\n", msg->data[0]);
        //printf("%hhu\n",msg->data[1]);
        //printf("%hhu\n",msg->data[2]);
        //printf("%hhu cell role\n",msg->data[3]);

        received_option_kilogrid = msg->data[2];// get the opinion of the tile
        wall_flag = msg->data[3];// if wall then 42, if near wall 62 else 0

        received_grid_msg_flag = true; //set the flag that message received from Kilogrid to true


        if (current_state == EXPLORATION){ //if the bot is in exploration state
            if(received_option_kilogrid != 0){ //if the the bot is not on the wall-white border
                if (foundmodules[msg->data[0]][msg->data[1]] == 0){ //if tile not counted previously

                    foundmodules[msg->data[0]][msg->data[1]] = 1; //set the flag that tile has been counted now
                    total_tiles_found += 1;

                    if(received_option_kilogrid==1){ //if I am Red and I receive red from kilogrid
                        tiles_of_1_option += MSG_KILOG ;
                        if(kilo_uid ==18) {
                            printf("%d tile my op 1,  %d total tiles  \n", tiles_of_1_option,
                                   total_tiles_found);
                        }
                    }else if(received_option_kilogrid==2) { //if I am blue and I receive blue from Kilogrid
                        tiles_of_2_option += MSG_KILOG ;
                        if(kilo_uid ==18) {

                            printf("%d tile my op 2  %d total tiles \n", tiles_of_2_option, total_tiles_found);
                        }
                    }else if(received_option_kilogrid==3 ){ //if I am blue and I receive blue from Kilogrid
                        tiles_of_3_option += MSG_KILOG ;
                        if(kilo_uid ==18) {

                            printf("%d tile my op 3,  %d total tiles \n", tiles_of_3_option,
                                   total_tiles_found);
                        }

                    }else if(received_option_kilogrid==4 ){ //if I am blue and I receive blue from Kilogrid
                        tiles_of_4_option += MSG_KILOG ;
                        if(kilo_uid ==18) {

                            printf("%d tile my op 4 ,  %d total tiles  \n", tiles_of_4_option,
                                   total_tiles_found);
                        }

                    }else if(received_option_kilogrid==5){ //if I am blue and I receive blue from Kilogrid
                        tiles_of_5_option += MSG_KILOG ;
                        if(kilo_uid ==18) {

                            printf("%d tile my op 5,  %d total tiles \n", tiles_of_5_option,
                                   total_tiles_found);
                        }

                    }


                }
            }
        }

        // printf("%hhu\n",msg->data[2]);
        // printf("%hhu\n",msg->data[3]);
        if (wall_flag == 62 || wall_flag == 42){  // robot sensed wall or near wall
            //  printf("received hitwall option");
            hit_wall = true; //-> set hit wall flag to true
            if(wall_flag == 62){ //if near the border of wall and not on white wall

                kilogrid_commitment = msg->data[2]; //still get the opinion from grid
                //printf("gets commitments\n");

            }
        }else{
            hit_wall = false; //not hitting the wall
            kilogrid_commitment = msg->data[2]; //get the opinion from kilogrid
        }

    }
    if(msg->type == INIT_MSG && !init_flag){
        // TODO add logic ...
        // example usage
//        kilogrid_commitment = msg->data[0];
//        my_commitment_quality = msg->data[1];
//        NUMBER_OF_OPTIONS = msg->data[2];
//        option_to_sample = rand() % NUMBER_OF_OPTIONS;
//        current_ground = msg->data[3];
//        communication_range = msg->data[4];

        init_flag = true;
    }else if(msg->type == GRID_MSG && init_flag){
        // TODO add logic ...
        //received_grid_msg_flag = true;
    }else if(msg->type == VIRTUAL_AGENT_MSG  && init_flag){
        // TODO add logic ...
        //received_virtual_agent_msg_flag = true;
    }
    return;
}


/*-----------------------------------------------------------------------------------------------*/
/* NOT USED: This function implements the sending to the kilogrid. you should call this function */
/* everyloop cycle because in reality you dont have a indicator if the message was received so we*/
/* have to send it multiple times. The when and how often to send a message should be            */
/* implemented here!                                                                             */
/*-----------------------------------------------------------------------------------------------
void message_tx(){

    // implementation differs because in simulation we use the debugstruct - faster and easier to
    // understand
    // in reality we send infrared msg - we send more than one to make sure that the messages arrive!
    if (msg_number_current_send != msg_number_send){
        msg_number_current_send = msg_number_send;
        msg_counter_sent = 0;
    }
#ifdef SIMULATION
    // reset broadcast flag - needed in simulation to stop sending messages
    if(msg_counter_sent == MSG_SEND_TRIES){
        debug_info_set(broadcast_flag, 0);
        msg_counter_sent += 1;
    }
#endif
    // send msg if not sent enough yet
    if (msg_counter_sent < MSG_SEND_TRIES){
#ifdef SIMULATION
        // count messages
        msg_counter_sent += 1;
#else
        if((message = kilob_message_send()) != NULL) {
            msg_counter_sent += 1;

        }
#endif
    }
}
*/
void message_tx_success(){ //if transmitted
    broadcast_msg = false; //set transmitted flag to false
    //set the colour

        set_color(RGB(2, 0, 2));
        delay(10);
        set_color(RGB(0, 0, 0));


}
message_t *message_tx()
{
    if( broadcast_msg ) { //if broadcast message flag is set to true (only in dissem state function)
        return &message;
    }
    return 0;
}

/*-----------------------------------------------------------------------------------------------*/
/* NOT USED: Setting values of the message                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void set_message(){
    // TODO this needs to be adjusted on what kind of messages you want to send: fill in data !
#ifdef SIMULATION
    msg_number_send += 1;
    debug_info_set(broadcast_flag, 1);
    debug_info_set(type, MSG_T_VIRTUAL_ROBOT_MSG);
    debug_info_set(data0, 1);
    debug_info_set(data1, 2);
    debug_info_set(data2, 3);
    debug_info_set(data3, 4);
    debug_info_set(data4, msg_number_send);
    debug_info_set(data5, 6);
    debug_info_set(data6, 7);
    debug_info_set(data7, 8);
#else
    // sample usage
    /*
    message->type = TO_KILOGRID_MSG;
    message->data[0] = kilogrid_commitment;
    message->data[1] = communication_range;
    message->data[2] = robot_gps_x;
    message->data[3] = robot_gps_y;
    message->data[4] = msg_number_current_send;
    */
#endif
}


/*-----------------------------------------------------------------------------------------------*/
/* Init function                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void setup(){


    srand(rand_hard());

    random_walk();

    //random timing for motion
    last_motion_ticks = rand() % max_straight_ticks + 1;

    //save the current ticks for comparison later on
    last_changed = kilo_ticks;
    message.type = FROMBOT; // set I am a bot
    // Quality A=1, B=2
    //////////currentopinion = (kilo_uid -1) % GROUPS +1;
    currentopinion = INFOPTION; //CODE FUNCTIONALITY TO CHOOSE RANDOMLY OF ANY 4 INFERIOR OPTIONS

    if(kilo_uid <kval){

        model = 1; //voter
    } else{

        model = 0; //majority
    }
    //if(kilo_uid % 2 == 0){ //choose muy opinion based on odd or even kilouid
       // currentopinion = 1;
    //}else{
      //  currentopinion = 2;
    //}

    // set parameters fro dissemination
    message.data[0] = currentopinion;
    //Opinion A=1 , B=2, U =3
    //red
    message.data[1] = currentopinion;
    message.data[2] = kilo_uid;
    message.crc = message_crc(&message);
    //timer =  ran_expo(1.0/avg_exploration_time); //get time to be in exploration state
    timer = avg_exploration_time;


    //kilogrid variables
#ifndef SIMULATION
    // for tracking the robot in real life
    kilob_tracking_init();
    kilob_messaging_init();
    tracking_data.byte[0] = kilo_uid;
    tracking_data.byte[5] = 0;
#endif

}


/*-----------------------------------------------------------------------------------------------*/
/* Main loop                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
void loop() {

    if(init_flag){  // initialization happened and messaged received from Kilogrid

        // if (received_grid_msg_flag) {
        //random_walk();
        //update_grid_msg();
        // check_if_against_a_wall();  // checks if the robot is on wall
        // received_grid_msg_flag = false;
        // }

        if (received_virtual_agent_msg_flag) {
            // update_virtual_agent_msg();
            received_virtual_agent_msg_flag = false;
        }

    }

    if ( wall_avoidance_state || hit_wall ) {
        wall_avoidance_function();
    } else {
        random_walk();
    }

    if (current_state == EXPLORATION){ // if state is set to 0


        gotoexploration(); //go to exploration

    }



    if (current_state == FILTER){  // state is set to choose between Vote or Noise

        //get the random number 0-1 to flip between self-sourced or social
       // double u = r2();
       poll();

    }



#ifdef SIMULATION
    // debug info - is now also important for the inter-robot communication, so do not delete
    debug_info_set(currentopinion, currentopinion);
    debug_info_set(estimateR, estimateR);
    debug_info_set(estimateB, estimateB);
    debug_info_set(estimateG, estimateG);
    debug_info_set(estimateY, estimateY);

    debug_info_set(estimateBL, estimateBL);
    debug_info_set(estimateN, estimateN);


    debug_info_set(tot_e, tot_e);
    debug_info_set(kilo_uid, kilo_uid);

#else
    tracking_data.byte[1] = received_x;
        tracking_data.byte[2] = received_y;
        tracking_data.byte[3] = com_range;
        tracking_data.byte[4] = msg_number_current;
        kilob_tracking(&tracking_data);
#endif
}


/*-----------------------------------------------------------------------------------------------*/
/* Main function - obviously needs to be implemented by both platforms.                          */
/*-----------------------------------------------------------------------------------------------*/
int main(){
    // initialize the hardware of the robot
    kilo_init();
    // now initialize specific things only needed for one platform
#ifdef SIMULATION
    // create debug struct - mimics the communication with the kilogrid
    debug_info_create();
#else
    utils_init();
#endif
    // callback for received messages
    kilo_message_rx = message_rx;
    // start control loop

    // Register the message_tx callback function.
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;
    kilo_start(setup, loop);
    return 0;
}
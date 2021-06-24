#include "kilolib.h"
//#define DEBUG
//#include "debug.h" // for real robots only
#include <stdio.h> // for ARGOS only
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "agentCDCIlocal.h"
#include <debug.h>

#define PI 3.14159265358979323846

#define UNCOMMITTED 0

#define	AGENT_MSG 21

/* Enum for different motion types */
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;

/* Flag for successful message sent */
bool message_sent = false;

/* Flag for decision to broadcast a message */
bool broadcast_msg = false;

/* Flag for ARK/Kbs Speaking Management */
bool broadcast_flag=true;

/* current motion type */
motion_t current_motion_type = STOP;

/* current LED color */
uint16_t current_LED_color=RGB(0,0,0);

/* current commitment */
uint8_t my_commitment=2;
uint8_t my_option_GPS_X;
uint8_t my_option_GPS_Y;
uint8_t my_option_quality;


/* counters for motion, turning, broadcasting and status-update */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t max_straight_ticks = 300;
const uint32_t broadcast_ticks = 15;
uint32_t last_motion_ticks = 0;
uint32_t last_broadcast_ticks = 0;
uint32_t update_ticks = 60; /* setting how often performing the commitment update. a tick here is every ~31ms */
uint32_t last_update_ticks = 0;

/* Variables for outgoing messages */
message_t message;

/* Variables for incoming messages from another robot */
uint8_t received_option_GPS_X;
uint8_t received_option_GPS_Y;
bool received_message;

/* Variables for incoming messages from ARK */
uint8_t discovered_option_GPS_X;
uint8_t discovered_option_GPS_Y;
uint8_t discovered_option_mean_quality;
uint8_t discovered_option_quality;
bool discovered;

/* Variables for Smart Arena messages */
unsigned int sa_type = 3;
unsigned int sa_payload = 0;
bool new_sa_msg_discovery = false;

/* Noise variables */
double standard_deviation=1.0;

/* Robot GPS variables */
uint8_t Robot_GPS_X;
uint8_t Robot_GPS_Y;
double Robot_orientation;
bool new_sa_msg_gps=false;

/* Robot Goal variables*/
uint8_t Goal_GPS_X;
uint8_t Goal_GPS_Y;
uint32_t lastWaypointTime;
uint32_t maxWaypointTime=3600; // about 2 minutes

/* Wall Avoidance manouvers */
uint32_t wallAvoidanceCounter=0; // to decide when the robot is stuck...

/* Options lookup table*/
uint8_t options_IDs[10];
uint8_t options_GPS_X[10];
uint8_t options_GPS_Y[10];
uint8_t number_of_options=0;

/* Global communication variables */
uint8_t redBots;
uint8_t greenBots;
uint8_t blueBots;
uint8_t redQ;
uint8_t greenQ;
uint8_t blueQ;
bool new_sa_msg_global_comm=false;

bool GoingToResampleOption=false;

/* RTID variables */
bool runtime_identification=false;
uint32_t backup_kiloticks;
uint16_t backup_LED;
motion_t backup_motion=STOP;


float RotSpeed=38.0;

uint8_t GPS_maxcell=16;
uint8_t minDist=4;

float GPS_To_Meter=1/16.0;
float Robot_FoV=0.0;

// model parameter
float param =0.01;

bool on_option=false;

bool debug_state=false;
uint32_t debug_lastTableUpdate = 0;
uint8_t debug_lastAssignedID = 0;
uint8_t debug_lastSource = 0;

/*-----------------------------*/
/*        Step function        */
/*-----------------------------*/
unsigned int step(float x){
    if (x>=0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*-------------------------------------------------------------------*/
/* Function to sample a random number form a Gaussian distribution   */
/*-------------------------------------------------------------------*/
double generateGaussianNoise(double mu, double std_dev )
{
    const double epsilon = DBL_MIN;
    const double two_pi = 2.0*PI;
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    double z0;
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    return z0 * std_dev + mu;
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(double* angle)
{
    while(*angle>180){
        *angle=*angle-360;
    }
    while(*angle<-180){
        *angle=*angle+360;
    }
}

/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
double AngleToGoal() {
    NormalizeAngle(&Robot_orientation);
    double angletogoal=atan2(Goal_GPS_Y-Robot_GPS_Y,Goal_GPS_X-Robot_GPS_X)/PI*180-Robot_orientation;
    NormalizeAngle(&angletogoal);
    return angletogoal;
}


/*-------------------------------------------------------------------*/
/* Coordinates to option ID                                          */
/*-------------------------------------------------------------------*/
uint8_t CoordsToID(uint8_t option_GPS_X,uint8_t option_GPS_Y)
{
    int i;
    for(i=0;i<number_of_options;i++){
        if( (option_GPS_X==options_GPS_X[i]) && (option_GPS_Y==options_GPS_Y[i]) )
        {
            return options_IDs[i];
        }
    }
    return UNCOMMITTED;
}

/*-------------------------------------------------------------------*/
/* Option ID to coordinates   (101 means 'ID not found')             */
/*-------------------------------------------------------------------*/
uint8_t IdToCoords(uint8_t id) {
    uint8_t i;
    for(i=0;i<number_of_options;i++){
        if( options_IDs[i]==id ) {
            return i;
        }
    }
    return 101;
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    if( current_motion_type != new_motion_type )
    {
        current_motion_type = new_motion_type;

        int calibrated = true;
        switch( new_motion_type )
        {
        case FORWARD:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated){

                if(!runtime_identification)
                    set_motors(kilo_straight_left,kilo_straight_right);
            }
            else
            {
                if(!runtime_identification)
                    set_motors(67,67);
            }
            break;
        case TURN_LEFT:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated)
            {
                if(!runtime_identification) {
                    uint8_t leftStrenght = kilo_turn_left;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            leftStrenght+=2;
                        }
                    }
                    set_motors(leftStrenght,0);
                }
            }
            else{
                if(!runtime_identification)
                    set_motors(70,0);
            }
            break;
        case TURN_RIGHT:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated){
                if(!runtime_identification) {
                    uint8_t rightStrenght = kilo_turn_right;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            rightStrenght+=2;
                        }
                    }
                    set_motors(0,rightStrenght);
                }
            }
            else{
                if(!runtime_identification)
                    set_motors(0,70);
            }
            break;
        case STOP:
        default:
            set_motors(0,0);
        }

        if(current_motion_type!=STOP){
            backup_motion=current_motion_type;
        }
    }
}


/*-------------------------------------------------------------------*/
/* Function for setting the the new commitment state                 */
/* (including LED colour and message initialisation)                 */
/*-------------------------------------------------------------------*/
void set_commitment( uint8_t new_option_GPS_X, uint8_t new_option_GPS_Y, uint8_t new_quality) {
    /* update the commitment state variable */
    my_option_GPS_X = new_option_GPS_X;
    my_option_GPS_Y = new_option_GPS_Y;
    my_option_quality = new_quality;
    my_commitment=CoordsToID(new_option_GPS_X,new_option_GPS_Y);
    debug_lastAssignedID=my_commitment;
}

/*-----------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model */
/*-----------------------------------------------------------------------------------*/
void random_walk_waypoint_model(bool selectNewWaypoint){
    /* if the robot arrived to the destination, OR too much time has passed, a new goal is selected */
    if ( selectNewWaypoint || ((Robot_GPS_X==Goal_GPS_X) && (Robot_GPS_Y==Goal_GPS_Y)) || kilo_ticks >= lastWaypointTime + maxWaypointTime) {
        lastWaypointTime = kilo_ticks;

        do {
            Goal_GPS_X=rand()%( GPS_maxcell-2 )+1; // getting a random number in the range [1,GPS_maxcell-1] to avoid the border cells (upper bound is -2 because index is from 0)
            Goal_GPS_Y=rand()%( GPS_maxcell-2 )+1;
            if ( abs(Robot_GPS_X-Goal_GPS_X) >= minDist || abs(Robot_GPS_Y-Goal_GPS_Y) >= minDist ){ // if the selected cell is enough distant from the current location, it's good
                break;
            }
        } while(true);
    }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise motors */
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    set_motion( FORWARD );
    last_motion_ticks = rand() % max_straight_ticks + 1;

    /* Initialise broadcast variables */
    last_broadcast_ticks = rand() % broadcast_ticks + 1;

    /** Initialise update variables */
    last_update_ticks= rand() % update_ticks;

    /* Initialise received message variables */
    received_message = false;
    received_option_GPS_X=0;
    received_option_GPS_Y=0;

    discovered = false;
    discovered_option_GPS_X=0;
    discovered_option_GPS_Y=0;
    discovered_option_quality = 0;

    /* Intialize time to 0 */
    kilo_ticks=0;

    /* initialise the GSP to the middle of the environment, to avoid to trigger wall avoidance immediately */
    Robot_GPS_X = GPS_maxcell/2;
    Robot_GPS_Y = GPS_maxcell/2;
    random_walk_waypoint_model(true);

}

/*-------------------------------------------------------------------------------*/
/* Function to check if the option has disappeared (to call at each GSP reading) */
/*-------------------------------------------------------------------------------*/
void check_if_my_option_has_disappeared()
{
    if(sqrt((Robot_GPS_X-my_option_GPS_X)*(Robot_GPS_X-my_option_GPS_X)+(Robot_GPS_Y-my_option_GPS_Y)*(Robot_GPS_Y-my_option_GPS_Y))*GPS_To_Meter < Robot_FoV && !on_option)
    {
        my_option_quality=0;

        if(GoingToResampleOption) {
            GoingToResampleOption=false;
            random_walk_waypoint_model(true);
        }
    }
}

/*-------------------------------------------------------------------*/
/* Simulating the receiption of another robot's message              */
/*-------------------------------------------------------------------*/
void update_virtual_global_communication() {
    new_sa_msg_global_comm=false;

    /* remove myself from the robot count */
    switch( my_commitment ) {
    case 1:
        if (redBots>0){ redBots = redBots-1; }
        break;
    case 2:
        if (blueBots>0){ blueBots = blueBots-1; }
        break;
    case 3:
        if (greenBots>0){ greenBots=greenBots-1; }
        break;
    default: break;
    }

    /* compute the number of robots signalling each option (i.e., number of robots by their broadcast probability) */
    double P_red   = redBots   * redQ   / 100.0;
    double P_green = greenBots * greenQ / 100.0;
    double P_blue  = blueBots  * blueQ  / 100.0;

    /* normalise each subpopulation by the signalling population size */
    double communicatingBots = P_red + P_green + P_blue;
    if (communicatingBots > 0){
        P_red = P_red/communicatingBots;
        P_green = P_green/communicatingBots;
        P_blue = P_blue/communicatingBots;
    } else {
        /* no communication made by any robot */
        P_red=0; P_green=0; P_blue=0;
        received_option_GPS_X = 0;
        received_option_GPS_Y = 0;
        received_message = false;
        return;
    }

    /* draw a random number */
    int randomInt = RAND_MAX;
    while (randomInt > 30000){
        randomInt = rand();
    }
    unsigned int RANGE_RND = 10000;
    unsigned int random = randomInt % RANGE_RND;

    /* convert the probability to int, in order to compare it with rand() int values */
    unsigned int P_redInt   = (unsigned int)(P_red  *RANGE_RND);
    unsigned int P_greenInt = (unsigned int)(P_green*RANGE_RND);
    unsigned int P_blueInt  = (unsigned int)(P_blue *RANGE_RND);

    uint8_t selectedOptID=0;
    if (P_red > 0 && random <= P_redInt) { /* virtually received a red message */
        selectedOptID = 1;
    } else {
        if (P_green > 0 && random <= (P_redInt+P_greenInt) ){ /* virtually received a green message */
            selectedOptID = 3;
        } else {
            if (P_blue > 0 && random <= (P_redInt+P_greenInt+P_blueInt) ){ /* virtually received a blue message */
                selectedOptID = 2;
            } else { /* this case should not happen */
                received_message = false;
                return;
            }
        }
    }

    uint8_t idx = IdToCoords(selectedOptID);
    if (idx > 100){ /* this case should not happen */
        received_message = false;
        return;
    } else {
        received_option_GPS_X = options_GPS_X[idx];
        received_option_GPS_Y = options_GPS_Y[idx];
        received_message = true;
    }

}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
    /** ARK MESSAGE **/
    if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0];
        int id2 = msg->data[3];
        int id3 = msg->data[6];

        if (id1 == kilo_uid)
        {
            // unpack type
            sa_type = (msg->data[1] >> 6) & 0x01;

            if(sa_type==0) // type 0 is GPS data
            {
                // get on_option flag
                on_option = msg->data[1] >> 7;

                // unpack payload
                Robot_GPS_X = msg->data[1]>>2 & 0x0F;
                Robot_GPS_Y = (msg->data[1] & 0x03) << 2 | (msg->data[2]>>6) ;
                Robot_orientation = (msg->data[2] & 0x3F)*12;

                new_sa_msg_gps = true;
            }
            else{  // otherwise it's discovery

                // get on_option flag
                on_option = msg->data[1] >> 7;

                // unpack payload
                discovered_option_GPS_X = msg->data[1]>>2 & 0x0F;
                discovered_option_GPS_Y = (msg->data[1] & 0x03)<< 2 | (msg->data[2]>>6) ;
                discovered_option_mean_quality = (msg->data[2] & 0x0F);

                new_sa_msg_discovery = true;
            }
        }
        if (id2 == kilo_uid)
        {
            // unpack type
            sa_type = (msg->data[4] >> 6) & 0x01;

            if(sa_type==0) // type 0 is GPS data
            {
                // get on_option flag
                on_option = msg->data[4] >> 7;

                // unpack payload
                Robot_GPS_X = msg->data[4]>>2 & 0x0F;
                Robot_GPS_Y = (msg->data[4] & 0x03) << 2 | (msg->data[5]>>6) ;
                Robot_orientation = (msg->data[5] & 0x3F)*12;

                new_sa_msg_gps = true;
            }
            else{ // otherwise it's discovery

                // get on_option flag
                on_option = msg->data[4] >> 7;

                // unpack payload
                discovered_option_GPS_X = msg->data[4]>>2 & 0x0F;
                discovered_option_GPS_Y = (msg->data[4] & 0x03)<< 2 | (msg->data[5]>>6) ;
                discovered_option_mean_quality = (msg->data[5] & 0x0F);

                new_sa_msg_discovery = true;
            }
        }
        if (id3 == kilo_uid)
        {
            // unpack type
            sa_type = (msg->data[7] >> 6) & 0x01;

            if(sa_type==0) // type 0 is GPS data
            {
                // get on_option flag
                on_option = msg->data[7] >> 7;

                // unpack payload
                Robot_GPS_X = msg->data[7]>>2 & 0x0F;
                Robot_GPS_Y = (msg->data[7] & 0x03) << 2 | (msg->data[8]>>6) ;
                Robot_orientation = (msg->data[8] & 0x3F)*12;

                new_sa_msg_gps = true;
            }
            else{ // otherwise it's discovery

                // get on_option flag
                on_option = msg->data[7] >> 7;

                // unpack payload
                discovered_option_GPS_X = msg->data[7]>>2 & 0x0F;
                discovered_option_GPS_Y = (msg->data[7] & 0x03)<< 2 | (msg->data[8]>>6) ;
                discovered_option_mean_quality = (msg->data[8] & 0x0F);

                new_sa_msg_discovery = true;
            }
        }
    }
    /** ARK Config File **/
    else if (msg->type == 10) {
        // Options lookup table

        if(number_of_options==0 || msg->data[0]!=options_IDs[number_of_options-1]){
            options_IDs[number_of_options] = msg->data[0];
            options_GPS_X[number_of_options] = msg->data[1];
            options_GPS_Y[number_of_options] = msg->data[2];
            Robot_FoV=(msg->data[3]/100.0)-GPS_To_Meter;

            minDist=msg->data[4];
            my_commitment=msg->data[5];
            debug_lastSource=77;
            debug_lastTableUpdate=kilo_ticks;

            discovered_option_quality = (uint8_t) ( 10 * (  generateGaussianNoise( msg->data[6],standard_deviation) ) );
            if(discovered_option_quality>100){
                discovered_option_quality=100;
            }
            if(discovered_option_quality<0){
                discovered_option_quality=0;
            }

            my_option_quality=discovered_option_quality;
            my_option_GPS_X=msg->data[7];
            my_option_GPS_Y=msg->data[8];

            number_of_options++;
        }
    }
    /** ARK ID identification **/
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    /** ARK Runtime identification **/
    else if (msg->type == 119) {
        // runtime identification
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id >= 0){ // runtime identification ongoing
            set_motion(STOP);
            runtime_identification = true;
            if (id == kilo_uid) {
                set_color(RGB(0,0,3));
            } else {
                set_color(RGB(3,0,0));
            }
        } else { // runtime identification ended
            kilo_ticks=backup_kiloticks;
            runtime_identification = false;
            set_motion(backup_motion);
        }
    }
    /** ARK Disable robot broadcasting **/
    else if (msg->type == 5) {
        broadcast_flag=false;
    }
    /** ARK Enable robot broadcasting **/
    else if (msg->type == 6) {
        broadcast_flag=true;
    }
    /** ARK Virtualisation of global communication by sending the population sizes **/
    else if (msg->type == 7) {
        redBots = msg->data[0];
        greenBots = msg->data[1];
        blueBots = msg->data[2];
        redQ = msg->data[3];
        greenQ = msg->data[4];
        blueQ = msg->data[5];
        new_sa_msg_global_comm=true;
    }
    /** Message from another robot **/
    else if (msg->type == AGENT_MSG) {
        // the received message is from another KB
        received_option_GPS_X = msg->data[0];
        received_option_GPS_Y = msg->data[1];
        received_message = (bool) msg->data[2];
    }

}

/*-----------------------------------------------------------------------------------------------------------------------------*/
/* Function to sample the quality value (with Gaussian noise applied to the virtual sensor 'perfect' reading).                 *
 * Updating the quality if it's my option, otherwise setting to true the discovery flag to be used in the update_commitment()  */
/*-----------------------------------------------------------------------------------------------------------------------------*/
void sample_option_quality(){
    new_sa_msg_discovery = false;
    discovered_option_quality = (uint8_t) ( 10 * (  generateGaussianNoise( discovered_option_mean_quality,standard_deviation) ) );
    if(discovered_option_quality>100){
        discovered_option_quality=100;
    }
    if(discovered_option_quality<0){
        discovered_option_quality=0;
    }
    if( ( discovered_option_GPS_X == my_option_GPS_X ) && ( discovered_option_GPS_Y == my_option_GPS_Y ) ) // re-sampling
    {
        set_commitment(my_option_GPS_X,my_option_GPS_Y,discovered_option_quality); // updating the quality with the latest estimated value
        debug_lastSource=99;

        if(GoingToResampleOption) {
            GoingToResampleOption=false;
            random_walk_waypoint_model(true); // after resampling the robot resume random walk
        }
        discovered = false;
    } else { // inside a site different from my_commitment
        if (GoingToResampleOption){
            discovered = false; // if going to re-sample my (new) commitment, do not make discovery
        } else {
            discovered = true; // discovery will be possible in the next update_commitment()
        }
    }
}


/*--------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message) */
/*--------------------------------------------------------------------------*/
void update_commitment() {
    if(runtime_identification) { return; }
    /* Updating the commitment only each update_ticks */
    if( kilo_ticks > last_update_ticks + update_ticks ) {
        last_update_ticks = kilo_ticks;
        /* drawing a random number */
        int randomInt = RAND_MAX;
        while (randomInt > 30000){
            randomInt = rand();
        }
        unsigned int RANGE_RND = 10000;
        unsigned int random = randomInt % RANGE_RND + 1;

        /* if the agent is uncommitted, it can do discovery or recruitment */
        if ( my_option_GPS_X==0 && my_option_GPS_Y==0){

            double P_discovery;
            bool social=false;
            bool individual=false;

            /* compute the transition probabilities as a fucntion of the estimated qualities */
            /* discovery is only possible if the robot has virtually sensed an option (via ARK) */
            if (discovered){
                P_discovery = discovered_option_quality / 100.0;
            } else {
                P_discovery = 0;
            }

            unsigned int P_discoveryInt = (unsigned int)(P_discovery*RANGE_RND)+1;
            /* DISCOVERY */
            if (P_discovery > 0 && random <= P_discoveryInt)
            {
                individual=true;
            }

            /* RECRUITMENT*/
            if (received_message && ( (received_option_GPS_X!=0) || (received_option_GPS_Y!=0) ))
            {
                social=true;
            }

            if(individual&&social)
            {
                if(rand()%2==0)
                {
                    individual=true;
                    social=false;
                }
                else{
                    individual=false;
                    social=true;
                }
            }

            if(individual)
            {
                /* the agent discovers a new option */
                set_commitment( discovered_option_GPS_X , discovered_option_GPS_Y , discovered_option_quality );
                debug_lastSource = 10;

                /* Resume random walk with a new waypoint */
                GoingToResampleOption=false;
                random_walk_waypoint_model(true);
            }

            if(social)
            {
                /* the agent gets recruited a new option*/
                set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                debug_lastSource = 20;
                Goal_GPS_X=my_option_GPS_X;
                Goal_GPS_Y=my_option_GPS_Y;
                GoingToResampleOption=true;
            }
        }
        /* if the agent is committed */
        else {

            double P_wary_scout;

            bool social=false;
            bool individual=false;

            if (discovered)
            {
                P_wary_scout = param*(discovered_option_quality/100.0);
            }
            else
            {
                P_wary_scout = 0;
            }

            unsigned int P_wary_scoutInt = (unsigned int)(P_wary_scout*RANGE_RND)+1;

            /* COMPARE */
            if (P_wary_scout > 0 && random <= P_wary_scoutInt)
            {
                individual=true;
            }

            /* Direct-switch */
            if(received_message && (received_option_GPS_X!=0) && (received_option_GPS_Y!=0) && ( (my_option_GPS_X!=received_option_GPS_X) || (my_option_GPS_Y!=received_option_GPS_Y) ) )
            {
                social=true;
            }


            if(individual&&social)
            {
                if(rand()%2==0)
                {
                    individual=true;
                    social=false;
                }
                else
                {
                    individual=false;
                    social=true;
                }
            }

            if(individual)
            {
                /* the agent switch to the new (and better) discovered option */
                set_commitment( discovered_option_GPS_X , discovered_option_GPS_Y , discovered_option_quality );
                debug_lastSource = 1;

                /* select a new waypoint for the random walk */
                random_walk_waypoint_model(true);
            }

            if(social)
            {
                /* the agent gets recruited a new option*/
                set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                debug_lastSource = 2;
                Goal_GPS_X=my_option_GPS_X;
                Goal_GPS_Y=my_option_GPS_Y;
                GoingToResampleOption=true;
            }
        }

        received_message = false;
        discovered = false;
    }
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk_open_loop(){
    switch( current_motion_type )
    {
    case TURN_LEFT:
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
            /* perform a radnom turn */
            last_motion_ticks = kilo_ticks;
            if( rand()%2 ) {
                set_motion(TURN_LEFT);
            }
            else {
                set_motion(TURN_RIGHT);
            }
            turning_ticks = rand()%max_turning_ticks + 1;
        }
        break;
    case STOP:
    default:
        set_motion(STOP);
    }
}


/*-------------------------------------------------------------------*/
/* Function to go to the Goal location (e.g. to resample an option)  */
/*-------------------------------------------------------------------*/
void GoToGoalLocation(){
    if(new_sa_msg_gps){
        new_sa_msg_gps=false;

        double angleToGoal = AngleToGoal();
        if(fabs(angleToGoal) <= 20)
        {
            set_motion(FORWARD);
            last_motion_ticks = kilo_ticks;
        }
        else{
            if(angleToGoal>0){
                set_motion(TURN_LEFT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
            }
            else{
                set_motion(TURN_RIGHT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
            }
        }
    }

    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
        }
        break;
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        break;

    case STOP:
    default:
        set_motion(STOP);
    }

}

/*-------------------------------------------------------------------*/
/* Function to broadcast the commitment message                     */
/*-------------------------------------------------------------------*/
void broadcast() {
    if ( ( kilo_ticks > last_broadcast_ticks + broadcast_ticks ) && (broadcast_flag==true) ) {
        last_broadcast_ticks = kilo_ticks;
        if ( my_option_GPS_X!=0 || my_option_GPS_Y!=0 )
        {
            /* Probabilistically decides to share or not share its commitement*/

            /* drawing a random number */
            int randomInt = RAND_MAX;
            while (randomInt > 30000){
                randomInt = rand();
            }
            unsigned int RANGE_RND = 10000;
            unsigned int random = randomInt % RANGE_RND + 1;

            double P_ShareCommitement=0;
            unsigned int P_ShareCommitementInt=0;

            P_ShareCommitement= ( my_option_quality / 100.0 );

            P_ShareCommitementInt= (unsigned int)(P_ShareCommitement*RANGE_RND)+1;;

            if (P_ShareCommitement > 0 && random <= P_ShareCommitementInt)
            {
                message.data[0] = my_option_GPS_X;
                message.data[1] = my_option_GPS_Y;
                message.data[2] = 1;
                message.type    = AGENT_MSG;
                message.crc     = message_crc(&message);

                /* set broadcast flag for transmission */
                broadcast_msg = true;
            }
            else
            {
                /* set broadcast flag for transmission */
                broadcast_msg = false;
            }
        }
        else {
            /* set broadcast flag for transmission */
            broadcast_msg = false;
        }
    }
    if (runtime_identification || !broadcast_flag) { broadcast_msg = false; }
}


/*-------------------------------------------------------------------*/
/* Callback function for message transmission                        */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
    if( broadcast_msg ) {
        return &message;
    }
    return 0;
}


/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void tx_message_success() {
    broadcast_msg = false;
}

void check_if_against_a_wall(){
    if ( Robot_GPS_X==0 || Robot_GPS_X>=GPS_maxcell-1 || Robot_GPS_Y==0 || Robot_GPS_Y>=GPS_maxcell-1 ){
        if (wallAvoidanceCounter<18)
            wallAvoidanceCounter += 1;
        else
            wallAvoidanceCounter = 1;
    } else {
        wallAvoidanceCounter = 0;
    }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
    if(!runtime_identification)
    {
        backup_kiloticks=kilo_ticks; // which we restore in after runtime_identification

        if (new_sa_msg_global_comm == true) {
            update_virtual_global_communication();
        }

        if (new_sa_msg_discovery == true) {
            sample_option_quality();
        }
        if (new_sa_msg_gps == true) {
            check_if_my_option_has_disappeared();
            check_if_against_a_wall();
        }

        if (!GoingToResampleOption && new_sa_msg_gps) { // if not going to resample
            random_walk_waypoint_model(false); // update the waypoints
        }
        GoToGoalLocation();

        update_commitment();
        broadcast();

        /* Set LED color depending on the robot's opinion*/

        if (number_of_options==3){
            switch( my_commitment ) {
            case 5:
                set_color(RGB(0,3,3));
                break;
            case 4:
                set_color(RGB(3,0,3));
                break;
            case 3:
                set_color(RGB(0,3,0));
                break;
            case 2:
                set_color(RGB(0,0,3));
                break;
            case 1:
                set_color(RGB(3,0,0));
                break;
            case 0:
                set_color(RGB(0,0,0));
                break;
            default:
                set_color(RGB(3,3,3));
                break;
            }
        }
        else{
            if (number_of_options<3){
                set_color(RGB(3,0,3)); // purple
            }
            if (number_of_options>3){
                if (debug_lastSource==77){
                    set_color(RGB(3,3,0)); // yellow
                } else {
                    if (my_commitment==0) {
                        set_color(RGB(3,3,3)); // white
                    } else {
                        set_color(RGB(0,3,3)); // ciano
                    }
                }
            }
        }
    }

    debug_info_set(commitement, my_commitment);
}


/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
    //    debug_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = tx_message_success;
    kilo_message_rx=message_rx;

    ////////////////////////////////////////
    // DEBUGGING INFORMATION
    //
    // Here is the place where you initialize the debugging information
    // struct.
    //
    // From now on, you can safely use 'debug_info' to refer to your
    // struct.
    debug_info_create();
    ////////////////////////////////////////

    kilo_start(setup, loop);
    return 0;
}

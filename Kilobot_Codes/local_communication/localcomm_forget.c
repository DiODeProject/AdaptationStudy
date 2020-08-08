#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "agentCDCIlocal.h"
#include <debug.h>
#include <float.h>

#define PI 3.14159265

#define UNCOMMITTED 0
//#define OPT_BLUE 1
//#define OPT_RED 2
//#define OPT_GREEN 3

#define	BEACON 77
#define	AGENT 21

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

/* current motion type */
motion_t current_motion_type = STOP;

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

// parameters
double h=1;
double k=1;
const double scaling = 0.032258;
double timeScaling;

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
double variance=0.1;

/* Robot GPS variables */
uint8_t Robot_GPS_X;
uint8_t Robot_GPS_Y;
double Robot_orientation;
bool new_sa_msg_gps=false;

/* Robot Goal variables*/
uint8_t Goal_GPS_X;
uint8_t Goal_GPS_Y;
bool GoingAway=false;


/* Options lookup table*/
uint8_t options_IDs[20];
uint8_t options_GPS_X[20];
uint8_t options_GPS_Y[20];
uint8_t number_of_options=0;

bool GoingToResampleOption=false;

uint8_t GPS_maxcell=16;
uint8_t minDist=4;

float GPS_To_Meter=1/16.0;
float Robot_FoV;

// model parameter
float param = 0.01;

bool on_option=false;
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
/* Function to generate a random nuber from a gaussian               */
/*-------------------------------------------------------------------*/
double generateGaussianNoise(double mu, double variance )
{
    const double epsilon = DBL_MIN;
    const double two_pi = 2.0*3.14159265358979323846;
    double sigma=sqrt(variance);
    //        double z1;
    //        bool generate;
    //        generate = !generate;

    //        if (!generate)
    //           return z1 * sigma + mu;
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    double z0;
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    //        z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(double* angle){
    if(*angle>180){
        *angle=*angle-360;
    }
    if(*angle<-180){
        *angle=*angle+360;
    }
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
double AngleToGoal() {
    double angletogoal=atan2(Goal_GPS_Y-Robot_GPS_Y,Goal_GPS_X-Robot_GPS_X)/PI*180-Robot_orientation;
    NormalizeAngle(&angletogoal);
    return angletogoal;
}


/*-------------------------------------------------------------------*/
/* Coordinates to option ID                                          */
/*-------------------------------------------------------------------*/
uint8_t CoordsToID(uint8_t option_GPS_X,uint8_t option_GPS_Y)
{
    for(int i=0;i<number_of_options;i++){
        if( (option_GPS_X==options_GPS_X[i]) && (option_GPS_Y==options_GPS_Y[i]) )
        {
            return options_IDs[i];
        }
    }
    return UNCOMMITTED;
}

/*-------------------------------------------------------------------*/
/* Coordinates to option ID  2                                        */
/*-------------------------------------------------------------------*/
uint8_t CoordsToID2(uint8_t X,uint8_t Y)
{
    for(int i=0;i<number_of_options;i++){
        if( (X==options_GPS_X[i]) && (Y==options_GPS_Y[i]) )
        {
            return i+1;
        }
    }

    return 0;
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    if( current_motion_type != new_motion_type ){
        int calibrated = true;
        switch( new_motion_type ) {
        case FORWARD:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_straight_left,kilo_straight_right);
            else
                set_motors(67,67);
            break;
        case TURN_LEFT:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_turn_left,0);
            else
                set_motors(70,0);
            break;
        case TURN_RIGHT:
            spinup_motors();
            if (calibrated)
                set_motors(0,kilo_turn_right);
            else
                set_motors(0,70);
            break;
        case STOP:
        default:
            set_motors(0,0);
        }
        current_motion_type = new_motion_type;
    }
}


/*-------------------------------------------------------------------*/
/* Function for setting the the new commitment state                 */
/* (including LED colour and message initialisation)                 */
/*-------------------------------------------------------------------*/
void set_commitment( uint8_t new_option_GPS_X, uint8_t new_option_GPS_Y, uint8_t new_quality) {

    /* update the commitment state varieable */
    my_option_GPS_X = new_option_GPS_X;
    my_option_GPS_Y = new_option_GPS_Y;
    my_option_quality = new_quality;
    my_commitment=CoordsToID(new_option_GPS_X,new_option_GPS_Y);
}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise commitment and LED */
    //    set_commitment(0 , 0 , 0);

    /* Initialise motors */
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    set_motion( FORWARD );
    last_motion_ticks = rand_soft() % max_straight_ticks + 1;

    /* Initialise broadcast variables */
    last_broadcast_ticks = rand_soft() % broadcast_ticks + 1;

    /** Initialise update variables */
    last_update_ticks= rand() % update_ticks;

    /* Initialise the scaling factor */
    timeScaling = scaling * 0.0333333 * update_ticks;

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

}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
    if (msg->type == 0) {
        // unpack message
        int id1 = msg->data[0];
        int id2 = msg->data[3];
        int id3 = msg->data[6];

        if (id1 == kilo_uid)
        {
            // unpack type
            sa_type = (msg->data[1] >> 6) & 0x01;

            if(sa_type==0)
            {
                // get on_option flag
                on_option = msg->data[1] >> 7;

                // unpack payload
                Robot_GPS_X = msg->data[1]>>2 & 0x0F;
                Robot_GPS_Y = (msg->data[1] & 0x03) << 2 | msg->data[2]>>6 ;
                Robot_orientation = (msg->data[2] & 0x3F)*12;

                NormalizeAngle(&Robot_orientation);
                //                debug_print("[t=%d] My GPS coords are: ( %d , %d ) \n",kilo_ticks, Robot_GPS_X , Robot_GPS_Y);
                //                debug_print("[t=%d] My orientation is: %f\n",kilo_ticks, Robot_orientation);
                new_sa_msg_gps = true;

                // Check if my option had disappeared
                if(sqrt((Robot_GPS_X-my_option_GPS_X)*(Robot_GPS_X-my_option_GPS_X)+(Robot_GPS_Y-my_option_GPS_Y)*(Robot_GPS_Y-my_option_GPS_Y))*GPS_To_Meter<Robot_FoV && !on_option)
                {


                    my_option_quality=0;
                    //                    if(my_commitment!=1){
                    //                        debug_print("[t=%d] My option %d disappeared!\n",kilo_ticks,my_commitment);
                    //                    }
                    if(GoingToResampleOption)
                    {
                        GoingToResampleOption=false;
                        GoingAway=true;

                        uint32_t distance=0;
                        do {
                            Goal_GPS_X=rand()%( GPS_maxcell-1 );
                            Goal_GPS_Y=rand()%( GPS_maxcell-1 );
                            distance=sqrt((discovered_option_GPS_X-Goal_GPS_X)*(discovered_option_GPS_X-Goal_GPS_X)+(discovered_option_GPS_Y-Goal_GPS_Y)*(discovered_option_GPS_Y-Goal_GPS_Y));
                        } while(distance<=minDist);
                    }

                    //                    debug_print("***********My Option Disappeared!!!\n");
                    //                    debug_print("Robot_GPS_X=%d  |  Robot_GPS_Y=%d\n",Robot_GPS_X,Robot_GPS_Y);
                    //                    debug_print("my_option_GPS_X=%d  |  my_option_GPS_X=%d\n",my_option_GPS_X,my_option_GPS_Y);
                    //                    debug_print("Distance=%f\n",sqrt((Robot_GPS_X-my_option_GPS_X)*(Robot_GPS_X-my_option_GPS_X)+(Robot_GPS_Y-my_option_GPS_Y)*(Robot_GPS_Y-my_option_GPS_Y))*GPS_To_Meter);
                    //                    debug_print("Option Flag=%d\n",msg->data[0] & 0x01);
                }
            }
            else{

                // get on_option flag
                on_option = msg->data[1] >> 7;

                // unpack payload
                discovered_option_GPS_X = msg->data[1]>>2 & 0x0F;
                discovered_option_GPS_Y = (msg->data[1] & 0x03)<< 2 | msg->data[2]>>6 ;
                discovered_option_mean_quality = (msg->data[2] & 0x0F);
                //                debug_print("[t=%d] Option GPS location is : ( %d , %d ) \n",kilo_ticks, discovered_option_GPS_X , discovered_option_GPS_Y);
                //                debug_print("[t=%d] on_option = %d\n",kilo_ticks, on_option);
                //                debug_print("[t=%d] Option mean quality is :  %d\n",kilo_ticks, discovered_option_mean_quality);
                new_sa_msg_discovery = true;
            }
        }

        if (id2 == kilo_uid)
        {
            //            sa_type = msg->data[4] >> 7;
            //            if(sa_type==0){
            //                // unpack payload
            //                Robot_GPS_X = msg->data[4]>>2 & 0x1F;
            //                Robot_GPS_Y = (msg->data[4] & 0x03)<< 3 | msg->data[5]>>5 ;
            //                Robot_orientation = (msg->data[5] & 0x1F)*12;
            //                NormalizeAngle(&Robot_orientation);
            //                new_sa_msg_gps = true;
            //            }
            //            else{
            //                // unpack payload
            //                discovered_option_GPS_X = msg->data[4]>>2 & 0x1F;
            //                discovered_option_GPS_Y = (msg->data[4] & 0x03)<< 3 | msg->data[5]>>5 ;
            //                discovered_option_mean_quality = (msg->data[5] & 0x1F);
            //                //                printf("My GPS coords are: ( %d , %d ) \n", Robot_GPS_X , Robot_GPS_Y);
            //                //                printf("My orientation is: %f\n", Robot_orientation);
            //                new_sa_msg_discovery = true;
            //            }
        }
        if (id3 == kilo_uid)
        {
            //            sa_type = msg->data[7] >> 7;
            //            if(sa_type==0){
            //                // unpack payload
            //                Robot_GPS_X = msg->data[7]>>2 & 0x1F;
            //                Robot_GPS_Y = (msg->data[7] & 0x03)<< 3 | msg->data[8]>>5 ;
            //                Robot_orientation = (msg->data[8] & 0x1F)*12;
            //                NormalizeAngle(&Robot_orientation);
            //                new_sa_msg_gps = true;
            //            }
            //            else{
            //                // unpack payload
            //                discovered_option_GPS_X = msg->data[7]>>2 & 0x1F;
            //                discovered_option_GPS_Y = (msg->data[7] & 0x03)<< 3 | msg->data[8]>>5 ;
            //                discovered_option_mean_quality = (msg->data[8] & 0x1F);
            //                //                  printf("My GPS coords are: ( %d , %d ) \n", Robot_GPS_X , Robot_GPS_Y);
            //                //                  printf("My orientation is: %f\n", Robot_orientation);
            //                new_sa_msg_discovery = true;
            //            }
        }
    }
    else if (msg->type == 1) { // Options lookup table
        options_IDs[number_of_options] = msg->data[0];
        options_GPS_X[number_of_options] = msg->data[1];
        options_GPS_Y[number_of_options] = msg->data[2];
        Robot_FoV=msg->data[3]/100.0-GPS_To_Meter;
        //        variance=msg->data[3]/10.0;
        //        debug_print("Robot_FoV=%f\n",Robot_FoV);
        //        GPS_maxcell=msg->data[4];
        //        debug_print("GPS_maxcell=%d\n",GPS_maxcell);
        minDist=msg->data[4];
        my_commitment=msg->data[5];

        discovered_option_quality = (uint8_t) ( 10 * (  generateGaussianNoise( msg->data[6],variance) ) );
        if(discovered_option_quality>100){
            discovered_option_quality=100;
        }
        if(discovered_option_quality<0){
            discovered_option_quality=0;
        }

        my_option_quality=discovered_option_quality;
        my_option_GPS_X=msg->data[7];
        my_option_GPS_Y=msg->data[8];


        //        debug_print("minDist=%d\n",minDist);
        //debug_print("I received option %d located at (%d,%d) \n",options_IDs[number_of_options], options_GPS_X[number_of_options], options_GPS_Y[number_of_options]);
        number_of_options++;
    }
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    //    else if (msg->type == AGENT && !GoingToResampleOption && !GoingAway) { // the received message is from another KB
//    else if (msg->type == AGENT && !GoingToResampleOption) { // the received message is from another KB
        else if (msg->type == AGENT) { // the received message is from another KB
        received_option_GPS_X = msg->data[0];
        received_option_GPS_Y = msg->data[1];
        received_message = (bool) msg->data[2];
        //        if(kilo_ticks>=70000){
        //            debug_print("[t=%d] **** received_option_GPS_X=%d -- received_option_GPS_Y=%d -- received_message=%d\n", kilo_ticks,received_option_GPS_X,received_option_GPS_Y,received_message);
        //        }
    }

    if (new_sa_msg_discovery == true) {
        new_sa_msg_discovery = false;
        discovered_option_quality = (uint8_t) ( 10 * (  generateGaussianNoise( discovered_option_mean_quality,variance) ) );
        if(discovered_option_quality>100){
            discovered_option_quality=100;
        }
        if(discovered_option_quality<0){
            discovered_option_quality=0;
        }

        if( ( discovered_option_GPS_X == my_option_GPS_X ) && ( discovered_option_GPS_Y == my_option_GPS_Y ) )
        {
            set_commitment(my_option_GPS_X,my_option_GPS_Y,discovered_option_quality);
            /*            if(my_option_quality!=0){
            // debug_print("Time[%d] - My commitement is option %d which has quality %d\n",kilo_ticks,my_commitment,my_option_quality);
                        }*/
            if(GoingToResampleOption)
            {
                GoingToResampleOption=false;
                GoingAway=true;

                uint32_t distance=0;
                do {
                    Goal_GPS_X=rand()%( GPS_maxcell-1 );
                    Goal_GPS_Y=rand()%( GPS_maxcell-1 );
                    distance=sqrt((discovered_option_GPS_X-Goal_GPS_X)*(discovered_option_GPS_X-Goal_GPS_X)+(discovered_option_GPS_Y-Goal_GPS_Y)*(discovered_option_GPS_Y-Goal_GPS_Y));
                } while(distance<=minDist);
            }
            return;
        }

        discovered = on_option;
    }

}



/*--------------------------------------------------------------------------*/
/* Function to normalise the quality from range [0,100] to range [0,255]    */
/*--------------------------------------------------------------------------*/
uint8_t normaliseQuality(uint8_t quality){
    double norm_quality = quality * 2.56;
    uint8_t norm_quality_ui = (uint8_t)(norm_quality);
    if (norm_quality > 255){ norm_quality_ui = 255; } //special case for quality = 100.
    return norm_quality_ui;
}

/*--------------------------------------------------------------------------*/
/* Function to update the parameters"k"                                     */
/*--------------------------------------------------------------------------*/
void update_individual_parameters()
{
    k=1.0;
}


/*--------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message) */
/*--------------------------------------------------------------------------*/
void update_commitment() {
    if(true){
        /* Updating the commitment only each update_ticks */
        if( kilo_ticks > last_update_ticks + update_ticks ) {
            update_individual_parameters();
            last_update_ticks = kilo_ticks;
            /* drawing a random number */
            int randomInt = RAND_MAX;
            while (randomInt > 30000){
                randomInt = rand();
            }
            unsigned int RANGE_RND = 10000;
            unsigned int random = randomInt % RANGE_RND + 1;

            /* if the agent is uncommitted, it can do discovery or recruitment */
            if ( my_option_GPS_X==0 && my_option_GPS_Y==0)
            {
                double P_discovery;
                bool social=false;
                bool individual=false;

                /* compute the transition probabilities as a fucntion of the estimated qualities */
                /* discovery is only possible if the message is received from a BEACON robot */
                if (discovered){
                    //                    P_discovery = timeScaling * k *(discovered_option_quality / 10.0);
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
                if (received_message && ( (received_option_GPS_X!=0) || (received_option_GPS_Y!=0) ) )
                {
                    social=true;
                }

                if(social&&individual)
                {
                    if(rand()%2==0){
                        social=true;
                        individual=false;
                    }
                    else
                    {
                        social=false;
                        individual=true;
                    }
                }

                if(individual)
                {
                    /* the agent discovers a new option */
                    set_commitment( discovered_option_GPS_X , discovered_option_GPS_Y , discovered_option_quality );

                    /* Go away from the discovered option */

                    uint32_t distance=0;
                    do {
                        Goal_GPS_X=rand()%( GPS_maxcell-1 );
                        Goal_GPS_Y=rand()%( GPS_maxcell-1 );
                        distance=sqrt((discovered_option_GPS_X-Goal_GPS_X)*(discovered_option_GPS_X-Goal_GPS_X)+(discovered_option_GPS_Y-Goal_GPS_Y)*(discovered_option_GPS_Y-Goal_GPS_Y));
                    } while(distance<=minDist);

                    GoingAway=true;
                }

                if(social)
                {
                    /* the agent discovers a new option*/
                    set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                    Goal_GPS_X=my_option_GPS_X;
                    Goal_GPS_Y=my_option_GPS_Y;
                    GoingToResampleOption=true;
                }
            }
            /* if the agent is committed */
            else
            {
                double P_abandon;
                bool social=false;
                bool individual=false;

                if (discovered)
                {
                    P_abandon = param;
//                    P_abandon = param/(discovered_option_quality / 100.0);
//                      P_abandon = (discovered_option_quality / 100.0)*param;
                }
                else{
                    P_abandon = 0;
                }

                unsigned int P_abandonInt = (unsigned int)(P_abandon*RANGE_RND)+1;
                /* COMPARE */
                if (P_abandon > 0 && random <= P_abandonInt)
                {
                    individual=true;
                }

                /* Direct-switch */
                if(received_message && (received_option_GPS_X!=0) && (received_option_GPS_Y!=0) && ( (my_option_GPS_X!=received_option_GPS_X) || (my_option_GPS_Y!=received_option_GPS_Y) ) )
                {
                    social=true;
                }

                if(social&&individual)
                {
                    if(rand()%2==0){
                        social=true;
                        individual=false;
                    }
                    else
                    {
                        social=false;
                        individual=true;
                    }
                }

                if(individual)
                {
                    /* the agent switch to the new (and better) discovered option */
                    set_commitment( 0 , 0 , 0 );
                }

                if(social)
                {
                    /* the agent discovers a new option*/
                    set_commitment(received_option_GPS_X,received_option_GPS_Y, 0);
                    Goal_GPS_X=my_option_GPS_X;
                    Goal_GPS_Y=my_option_GPS_Y;
                    if(GoingAway)
                    {
                        GoingAway=false;
                    }
                    GoingToResampleOption=true;
                }
            }
            received_message = false;
            discovered = false;
        }
    }
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
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
            if( rand_soft()%2 ) {
                set_motion(TURN_LEFT);
            }
            else {
                set_motion(TURN_RIGHT);
            }
            turning_ticks = rand_soft()%max_turning_ticks + 1;
        }
        break;
    case STOP:
    default:
        set_motion(STOP);
    }
}


/*-------------------------------------------------------------------*/
/* Function to go resample an option                                 */
/*-------------------------------------------------------------------*/
void GoToOption(){
    if(new_sa_msg_gps){
        new_sa_msg_gps=false;

        //debug_print("My orientation is: %f\n", Robot_orientation);
        //debug_print("In need to turn: %f\n", AngleToGoal() );

        if(( (Robot_GPS_X==Goal_GPS_X) &&  (Robot_GPS_Y==Goal_GPS_Y) ) || ( GoingAway && ( Robot_GPS_X==0 || Robot_GPS_X>=GPS_maxcell-1 || Robot_GPS_Y==0 || Robot_GPS_Y>=GPS_maxcell-1 ) ) )
        {
            //            set_motion(STOP);
            if( GoingAway){
                //                set_motion(STOP);
                GoingAway=false;
                //            set_motion(FORWARD);
                //            last_motion_ticks = kilo_ticks+8*max_straight_ticks;
                //                set_color(RGB(0,0,0));
                //                debug_print("I reached my goal!\n");
            }
            //            set_color(RGB(0,0,0));
        }
        else{
            if(fabs(AngleToGoal()) <= 20)
            {

                set_motion(FORWARD);
                last_motion_ticks = kilo_ticks;
            }
            else{
                if(AngleToGoal()>0){
                    set_motion(TURN_LEFT);
                    last_motion_ticks = kilo_ticks;
                    turning_ticks=(unsigned int) ( fabs(AngleToGoal())/45.0*30.0 );
                    //                    debug_print("In need to turn left for: %d\n", turning_ticks );
                }
                else{
                    set_motion(TURN_RIGHT);
                    last_motion_ticks = kilo_ticks;
                    turning_ticks=(unsigned int) ( fabs(AngleToGoal())/45.0*30.0 );
                    //                    debug_print("In need to turn right for: %d\n", turning_ticks );
                }
            }
        }
    }

    switch( current_motion_type ) {
    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
        break;
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
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

/*--------------------------------------------------------------------------*/
/* Function to update the parameters "h" and "k"                            */
/*--------------------------------------------------------------------------*/
void update_social_parameters()
{
    h=1.0;
}

/*-------------------------------------------------------------------*/
/* Function to broadcast the commitment message                     */
/*-------------------------------------------------------------------*/
void broadcast() {
    if( kilo_ticks > last_broadcast_ticks + broadcast_ticks ) {
        last_broadcast_ticks = kilo_ticks;
        if ( my_option_GPS_X!=0 || my_option_GPS_Y!=0 )
        {
            /* Probabilistically decides to share or not share its commitement*/

            update_social_parameters();

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
                message.type    = AGENT;
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


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {

    if(GoingToResampleOption || GoingAway){
        GoToOption();
    }
    else{
        random_walk();
    }

    update_commitment();
    broadcast();

    /* Set LED color*/
    //    if(GoingToResampleOption || GoingAway){
    //        set_color(RGB(3,0,0));
    //    }
    //    else{

    switch( my_commitment ) {
    case 5:
        set_color(RGB(0,3,3));
        break;
    case 4:
        set_color(RGB(3,0,3));
        break;
    case 3:
        set_color(RGB(0,0,3));
        break;
    case 2:
        set_color(RGB(0,3,0));
        break;
    case 1:
        set_color(RGB(3,0,0));
        break;
    case 0:
    default:
        set_color(RGB(0,0,0));
        break;
    }

    debug_info_set(commitement, my_commitment);
}


/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
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

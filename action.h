#ifndef ACTION_H
#define ACTION_H

enum actionMode{
    DISABLE,                /*CONTTROLLER OFF*/
    HOVER,                  /*BASE FUNCTION + PID for VELOCITY and DISPLACEMENT WITH VELOCITY TARGET 0*/
    DUAL_CORRECTION,        /*BASE FUNCTION + PID for VELOCITY and DISPLACEMENT independently*/
    VELOCITY_NO_CORRECTION, /*velocity mode BASE FUNCTION only*/
    VELOCITY_CORRECTION,    /*velocity mode BASE FUNCTION + PID for VELOCITY*/
    DISPLACEMENT_CORRECTION,/*displacement mode PID for DISPLACEMENT*/
    
};

struct action{
double target_velocity=0;
double target_displacement=0;
double actual_velocity=0;
double actual_displacement=0;
};

#endif
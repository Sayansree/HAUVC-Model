#ifndef ACTION_H
#define ACTION_H

enum actionMode{
    DISABLE,                /*CONTTROLLER/THRUSTERS OFF*/
    
    HOVER,                  /*BASE FUNCTION + PID for VELOCITY and DISPLACEMENT WITH VELOCITY TARGET 0*/
    HYBRID,                 /*BASE FUNCTION + PID for VELOCITY and DISPLACEMENT*/
    DUAL_CORRECTION,        /* PID for VELOCITY and DISPLACEMENT independently*/
    
    //velocity control
    VELOCITY_CORRECTION,    /*velocity mode (requires DVL) BASE FUNCTION + PID for VELOCITY*/
    VELOCITY_NO_CORRECTION, /*velocity mode BASE FUNCTION only*/
    VELOCITY_NO_BASE,       /*velocity mode (requires DVL) PID for VELOCITY*/
    
    //displacement control
    DISPLACEMENT,           /*displacement mode PID for DISPLACEMENT*/
    
};

struct action{
double target_velocity=0;
double target_displacement=0;
double actual_velocity=0;
double actual_displacement=0;
};

#endif
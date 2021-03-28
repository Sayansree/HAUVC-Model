#ifndef ACTION_H
#define ACTION_H

enum actionMode{
    DISABLE,
    NO_CORRECTION,
    VELOCITY_CORRECTION,
    DISPLACEMENT_CORRECTION,
    HOVER,
    DUAL_CORRECTION
};

struct action{
double target_velocity=0;
double target_displacement=0;
double actual_velocity=0;
double actual_displacement=0;
};

#endif
#ifndef MODEL_H
#define MODEL_H

enum actionMode{
    disable,
    noCorrection,
    velocityCorrection,
    displacementCorrection,
    Hover,
    DualCorrection
};

struct action{
double target_velocity;
double target_displacement;
double actual_velocity;
double actual_displacement;
actionMode mode;
};
#endif
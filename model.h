#ifndef MODEL_H
#define MODEL_H

#include "action.h"
#include "PID-Controller/PID.h"

class model{
    public:
        model();
        ~model();
        double update(action);
        void setMode(actionMode);
        

    private:
        actionMode mode;
        PID *displacementController,*velocityController;
        double baseFunction(double);
        double trim(double, double, double);
        double a0,a1,a2;

};
#endif
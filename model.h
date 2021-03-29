/* written by Sayansree Paria
 * email : sayansreeparia@gmail.com
 * github : https://github.com/Sayansree
 */
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
        void setCoeff(double,double,double);
        void setVelWeights(double,double,double);
        void setDispWeights(double,double,double);
        void setILimits(double,double);
    private:
        actionMode mode;
        PID *displacementController,*velocityController;
        double baseFunction(double);
        double trim(double, double, double);
        double a0,a1,a2;

};
#endif
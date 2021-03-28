#include "model.h"

model::model(){
    displacementController=new PID();
    velocityController=new PID();
    mode=DISABLE;
}
model::~model(){

}
double PID::trim(double val, double min, double max){
    if(val>max)return max;
    if(val<min)return min;
    return val;
}
void model::setMode(actionMode mode){
    if(this->mode==mode)return;
    this->mode=mode;
    displacementController->reset();
    velocityController->reset();
}
double model::update(action cmd){
    switch(mode){
        case DISABLE:
            return 0;
        case VELOCITY_NO_CORRECTION:
            return baseFunction(cmd.target_velocity);
        case VELOCITY_CORRECTION:
            double baseFunc=baseFunction(cmd.target_velocity);
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            double pidVel=velocityController->update(errorVel);
            return trim(pidVel+baseFunc,-1,1);
        case DISPLACEMENT_CORRECTION:
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            return displacementController->update(errorDisp);
        case DUAL_CORRECTION:
            double baseFunc=baseFunction(cmd.target_velocity);
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            double pidVel=velocityController->update(errorVel);
    }
}
double model::baseFunction(double velocity){
   return a2*velocity*velocity + a1*velocity + a0;
}
int main (){
    return 0;
}
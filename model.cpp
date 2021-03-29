#include "model.h"

model::model(){
    displacementController=new PID();
    velocityController=new PID();
    displacementController->setPIDLimits(-1,1);
    velocityController->setPIDLimits(-1,1);
    mode=DISABLE;
}
model::~model(){

}
void model::setILimits(double lo,double hi){
    velocityController->setILimits(lo,hi);
    displacementController->setILimits(lo,hi);
}
void model::setVelWeights(double Kp,double Ki,double Kd){
    velocityController->setWeights(Kp,Ki,Kd);
}
void model::setDispWeights(double Kp,double Ki,double Kd){
    displacementController->setWeights(Kp,Ki,Kd);
}
void model::setCoeff(double a2,double a1,double a0){
    this->a2=a2;
    this->a1=a1;
    this->a0=a0;
}
double model::trim(double val, double min, double max){
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

        case HOVER:{
            double baseFunc=baseFunction(0);
            double errorVel=0-cmd.actual_velocity;
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            double pidVel=velocityController->update(errorVel);
            double pidDisp= displacementController->update(errorDisp);
            return trim(pidVel+pidDisp+baseFunc,-1,1);}
        case HYBRID:{
            double baseFunc=baseFunction(cmd.actual_velocity);
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            double pidVel=velocityController->update(errorVel);
            double pidDisp= displacementController->update(errorDisp);
            return trim(pidVel+pidDisp+baseFunc,-1,1);}
        case SINGLE_CORRECTION:{
            double baseFunc=baseFunction(cmd.actual_velocity);
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            double pidDisp= displacementController->update(errorDisp);
            return trim(baseFunc+pidDisp,-1,1);}
        case DUAL_CORRECTION:{
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            double pidVel=velocityController->update(errorVel);
            double pidDisp= displacementController->update(errorDisp);
            return trim(pidVel+pidDisp,-1,1);}

        case VELOCITY_NO_CORRECTION:
            return baseFunction(cmd.target_velocity);
        case VELOCITY_NO_BASE:{
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            return velocityController->update(errorVel);}
        case VELOCITY_CORRECTION:{
            double baseFunc=baseFunction(cmd.target_velocity);
            double errorVel=cmd.target_velocity-cmd.actual_velocity;
            double pidVel=velocityController->update(errorVel);
            return trim(pidVel+baseFunc,-1,1);}
        
        case DISPLACEMENT:{
            double errorDisp=cmd.target_displacement-cmd.actual_displacement;
            return displacementController->update(errorDisp);}
    }
}
double model::baseFunction(double velocity){
   return a2*velocity*velocity + a1*velocity + a0;
}
int main (){
    return 0;
}
#include "model.h"

model::model(){
    displacementController=new PID();
    velocityController=new PID();
    mode=DISABLE;
}
model::~model(){

}
double model::update(action cmd){
    switch(mode){
        case DISABLE:
            return 0;
            break;
        case NO_CORRECTION:

    }
}
double model::baseFunction(double velocity){
   return a2*velocity*velocity + a1*velocity + a0;
}
int main (){
    return 0;
}
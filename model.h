#ifndef MODEL_H
#define MODEL_H

#include "action.h"
#include "PID-Controller/PID.h"




class model{
    public:
        model();
        ~model();
        double update(action);
        

    private:
        

};
#endif
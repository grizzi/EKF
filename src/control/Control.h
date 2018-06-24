//
// Created by giuseppe on 09/06/18.
//

#ifndef EKF_CONTROL_H
#define EKF_CONTROL_H

#include <cassert>
class Controller
{
    int type_{0};
    double A_{2.0};
    double omega_{0.5};

public:
    Controller(){}

    void setType(int type)
    {
        assert(type==0 || type==1);
        type_ = type;
    }

    void setAmplitude(double A){ A_ = A;}

    void setFrequency(double &omega){ omega_ = omega;}

    double sinInput(double &t){ return A_*sin(omega_*t);}

    double Step(){return A_;}

    double getNext(double &time)
    {
        switch(type_)
        {
            case 0:
                return Step();
            case 1:
                return sinInput(time);
            default:
                return Step();
        }
    }
};
#endif //EKF_CONTROL_H

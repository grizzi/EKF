//
// Created by giuseppe on 05/06/18.
//

#ifndef SIMULATOR_DYNAMICS_H
#define SIMULATOR_DYNAMICS_H

#include <iostream>
#include <iomanip>
#include <boost/numeric/odeint.hpp>
#include <boost/circular_buffer.hpp>
#include <cassert>
#include "utils.h"

using namespace std;
using namespace boost::numeric::odeint;


typedef  std::vector<double> State;


class Observer
{
public:
    boost::circular_buffer< State >& m_states;
    boost::circular_buffer< double >& m_times;

    Observer( boost::circular_buffer< State >& states , boost::circular_buffer< double >& times)
            : m_states( states ) , m_times( times ) { }

    void operator()( const State &x , double t );
};

class Dynamics
{

public:
    State pos_;
    State speed_;
    int dim_;
    double timer_{0.0};

    // Buffers
    boost::circular_buffer<State> x_vector_;
    boost::circular_buffer<double> times_;

    /* Default constructor*/
    Dynamics(){}

    /********************* Constructor **************************************/
    Dynamics(int dim, State &ic);

    /********************* System ODE **************************************/
    virtual void sys(const State &x, State &dxdt, double t){}

    /********************** System Integrator ******************************/
    void step(double delta_t);


    /*************************** Get methods******************************/
    State getPos();

    State getSpeed();


    double getPos(int );            // Optional retrieval of a specific state

    double getSpeed(int );

    double getInternalTime();

    /**************************** Print methods***************************/
    void printPos();

    void printSpeed();

};


#endif //SIMULATOR_DYNAMICS_H



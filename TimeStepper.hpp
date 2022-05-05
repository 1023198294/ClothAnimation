#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "vecmath.h"
#include <vector>
#include "particleSystem.h"
#include "Condition.h"
#include "CGSolver.h"
#include <Eigen/Core>
#include <Eigen/Dense>

class TimeStepper {
public:
    virtual void takeStep(ParticleSystem *particleSystem, float stepSize) = 0;

};

//IMPLEMENT YOUR TIMESTEPPERS

class ForwardEuler : public TimeStepper {
    void takeStep(ParticleSystem *particleSystem, float stepSize);
};

class Trapzoidal : public TimeStepper {
    void takeStep(ParticleSystem *particleSystem, float stepSize);
};

/////////////////////////

//Provided
class RK4 : public TimeStepper {
    void takeStep(ParticleSystem *particleSystem, float stepSize);
};

class ode45 : public TimeStepper {
    void takeStep(ParticleSystem *particleSystem, float stepSize);

private:
    float step_size;
    float tolerance;
};


class ImplicitSolver : public TimeStepper {
    void takeStep(ParticleSystem *particleSystem, float stepSize);
};

#endif

#include "TimeStepper.hpp"

///TODO: implement Explicit Euler time integrator here
void ForwardEuler::takeStep(ParticleSystem *particleSystem, float stepSize) {

    vector<Vector3f> x_current = particleSystem->getState();
    vector<Vector3f> FVal = particleSystem->evalF(x_current);
    vector<Vector3f> x_next;
    x_next.reserve(x_current.size());
    for (int i = 0; i < x_current.size(); i++) {
        x_next.push_back(x_current[i] + stepSize * FVal[i]);
    }
    particleSystem->setState(x_next);

}

void Trapzoidal::takeStep(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f> x_current = particleSystem->getState();
    vector<Vector3f> FVal_0 = particleSystem->evalF(x_current);
    vector<Vector3f> FVal_1;
    FVal_1.reserve(x_current.size());
    vector<Vector3f> x_next_1;
    x_next_1.reserve(x_current.size());
    vector<Vector3f> x_next;
    x_next.reserve(x_current.size());
    for (int i = 0; i < FVal_0.size(); i++) {
        x_next_1.push_back(x_current[i] + stepSize * FVal_0[0]);
    }
    for (int i = 0; i < x_next_1.size(); i++) {
        FVal_1.push_back(x_current[i] + stepSize * (FVal_0[i] + FVal_1[i]) / 2.0);
    }
    particleSystem->setState(FVal_1);
}

///TODO: implement RK4 here
void RK4::takeStep(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f> x_current = particleSystem->getState();
    vector<Vector3f> state_f1 = particleSystem->evalF(x_current);
    vector<Vector3f> state_f2 = x_current;
    vector<Vector3f> state_f3 = x_current;
    vector<Vector3f> state_f4 = x_current;
    vector<Vector3f> k1 = x_current;
    vector<Vector3f> k2 = x_current;
    vector<Vector3f> k3 = x_current;
    vector<Vector3f> k4 = x_current;
    for (int i = 0; i < x_current.size(); i++) {
        k1[i] = stepSize * state_f1[i];
        state_f2[i] = state_f2[i] + 0.5 * k1[i];
    }
    vector<Vector3f> FVal2 = particleSystem->evalF(state_f2);
    for (int i = 0; i < x_current.size(); i++) {
        k2[i] = stepSize * FVal2[i];
        state_f3[i] = state_f3[i] + (0.5) * k2[i];
    }
    vector<Vector3f> FVal3 = particleSystem->evalF(state_f3);
    for (int i = 0; i < x_current.size(); i++) {
        k3[i] = stepSize * FVal3[i];
        state_f4[i] = state_f4[i] + k3[i];
    }
    vector<Vector3f> FVal4 = particleSystem->evalF(state_f4);
    for (int i = 0; i < x_current.size(); i++) {
        k4[i] = stepSize * FVal4[i];
        x_current[i] = x_current[i] + (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    particleSystem->setState(x_current);
}

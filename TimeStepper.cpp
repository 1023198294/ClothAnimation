#include <iostream>
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
    vector<Vector3f> newX;

    vector<Vector3f> currentX = particleSystem->getState();

    vector<Vector3f> f_0 = particleSystem->evalF(currentX);
    vector<Vector3f> f_1_x;

    for (int x=0; x<f_0.size(); x++){
        Vector3f X_each = currentX[x] + (f_0[x]*stepSize);
        f_1_x.push_back(X_each);
    }

    vector<Vector3f> f_1 = particleSystem->evalF(f_1_x);

    for (int i=0; i<currentX.size(); i++){
        Vector3f eachX = currentX[i] + ((f_0[i] + f_1[i])*(stepSize/2.0));
        newX.push_back(eachX);
    }

    particleSystem->setState(newX);
}

vector<Vector3f> RK4_step(ParticleSystem *particleSystem, float stepSize) {
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
    return x_current;
}

///TODO: implement RK4 here
void RK4::takeStep(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f> x_current = RK4_step(particleSystem, stepSize);
    particleSystem->setState(x_current);
}

//float Vector3f::abs() const
//{
//    return sqrt( m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] );
//}
void ode45::takeStep(ParticleSystem *particleSystem, float stepSize) {
    step_size = stepSize;
    float safety_factor = 0.9;
    tolerance = 0.01;
    float error = 10000.0;
    vector<Vector3f> x1;
    x1 = RK4_step(particleSystem, step_size);
    while (error > tolerance) {
        x1 = RK4_step(particleSystem, step_size);
        vector<Vector3f> x2 = RK4_step(particleSystem, step_size / 2);
        error = 0;
        for (int j = 0; j < x1.size(); j++) {
            error += sqrt((
                                  x1[j].x() - x2[j].x()) * (x1[j].x() - x2[j].x()) +
                          (x1[j].y() - x2[j].y()) * (x1[j].y() - x2[j].y()) +
                          (x1[j].z() - x2[j].z()) * (x1[j].z() - x2[j].z()));
        }
        error /= x1.size();
//        error = tolerance;
        float new_time_step = safety_factor * step_size * sqrt(sqrt(tolerance / error));
//        cout << "ode error: " << error << " old time: " << step_size << " new step: " << new_time_step << endl;
        step_size = new_time_step;
    }
    particleSystem->setState(x1);
}

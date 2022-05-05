#include <iostream>
#include "TimeStepper.hpp"

///TODO: implement Explicit Euler time integrator here
void ForwardEuler::takeStep(ParticleSystem *particleSystem, float stepSize) {

    vector<Vector3f_> x_current = particleSystem->getState();
    vector<Vector3f_> FVal = particleSystem->evalF(x_current);
    vector<Vector3f_> x_next;
    x_next.reserve(x_current.size());
    for (unsigned int i = 0; i < x_current.size(); i++) {
        x_next.push_back(x_current[i] + stepSize * FVal[i]);
    }
    particleSystem->setState(x_next);

}

void Trapzoidal::takeStep(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f_> x_current = particleSystem->getState();
    vector<Vector3f_> FVal_0 = particleSystem->evalF(x_current);
    vector<Vector3f_> FVal_1;
    FVal_1.reserve(x_current.size());
    vector<Vector3f_> x_next_1;
    x_next_1.reserve(x_current.size());
    vector<Vector3f_> x_next;
    x_next.reserve(x_current.size());
    for (unsigned int i = 0; i < FVal_0.size(); i++) {
        x_next_1.push_back(x_current[i] + stepSize * FVal_0[0]);
    }
    for (unsigned int i = 0; i < x_next_1.size(); i++) {
        FVal_1.push_back(x_current[i] + stepSize * (FVal_0[i] + FVal_1[i]) / 2.0);
    }
    particleSystem->setState(FVal_1);
}

vector<Vector3f_> RK4_step(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f_> x_current = particleSystem->getState();
    vector<Vector3f_> state_f1 = particleSystem->evalF(x_current);
    vector<Vector3f_> state_f2 = x_current;
    vector<Vector3f_> state_f3 = x_current;
    vector<Vector3f_> state_f4 = x_current;
    vector<Vector3f_> k1 = x_current;
    vector<Vector3f_> k2 = x_current;
    vector<Vector3f_> k3 = x_current;
    vector<Vector3f_> k4 = x_current;
    for (unsigned int i = 0; i < x_current.size(); i++) {
        k1[i] = stepSize * state_f1[i];
        state_f2[i] = state_f2[i] + 0.5 * k1[i];
    }
    vector<Vector3f_> FVal2 = particleSystem->evalF(state_f2);
    for (unsigned int i = 0; i < x_current.size(); i++) {
        k2[i] = stepSize * FVal2[i];
        state_f3[i] = state_f3[i] + (0.5) * k2[i];
    }
    vector<Vector3f_> FVal3 = particleSystem->evalF(state_f3);
    for (unsigned int i = 0; i < x_current.size(); i++) {
        k3[i] = stepSize * FVal3[i];
        state_f4[i] = state_f4[i] + k3[i];
    }
    vector<Vector3f_> FVal4 = particleSystem->evalF(state_f4);
    for (unsigned int i = 0; i < x_current.size(); i++) {
        k4[i] = stepSize * FVal4[i];
        x_current[i] = x_current[i] + (1.0 / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    return x_current;
}

///TODO: implement RK4 here
void RK4::takeStep(ParticleSystem *particleSystem, float stepSize) {
    vector<Vector3f_> x_current = RK4_step(particleSystem, stepSize);
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
    vector<Vector3f_> x1;
    x1 = RK4_step(particleSystem, step_size);
    while (error > tolerance) {
        x1 = RK4_step(particleSystem, step_size);
        vector<Vector3f_> x2 = RK4_step(particleSystem, step_size / 2);
        error = 0;
        for (unsigned int j = 0; j < x1.size(); j++) {
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

Eigen::Vector3f vecmathVecToEigenVec(Vector3f_ vec){
    Eigen::Vector3f v;
    float x = vec[0];
    float y = vec[1];
    float z = vec[2];
    v << x, y, z;
    return v;
}

Vector3f_ eigenVecToVecmathVec(Eigen::Vector3f vec){
    float x = vec(0);
    float y = vec(1);
    float z = vec(2);
    Vector3f_ v(x, y, z);
    return v;
}

void ImplicitSolver::takeStep(ParticleSystem *particleSystem, float stepSize){
    vector<Vector3f_> state = particleSystem->getState(); // (pos, vel)
    vector<Vector3f_> FVal = particleSystem->evalF(state); // (vel, F/M, fixed)
    vector<Vector3f_> newState;

    lhs = Eigen::MatrixXf::Zero(particleSystem->m_numParticles * 3, particleSystem->m_numParticles * 3);
    rhs = Eigen::VectorXf::Zero(particleSystem->m_numParticles * 3);

    Eigen::VectorXf vel = Eigen::VectorXf::Zero(particleSystem->m_numParticles * 3);
    Eigen::VectorXf result = Eigen::VectorXf::Zero(particleSystem->m_numParticles * 3);
    // Identity matrices
    for(int i = 0; i < particleSystem->m_numParticles; i++){
        lhs.block<3, 3>(i * 3, i * 3) = Eigen::Matrix3f::Identity(3, 3);
        vel.segment<3>(i * 3) = vecmathVecToEigenVec(FVal[3 * i]);
        // update forces
        forces.segment<3>(i * 3) = vecmathVecToEigenVec(FVal[3 * i + 1]);
    }

    lhs = lhs - (stepSize * dfdv / 2.0f) - ((stepSize * stepSize / 2.0f)  * dfdx);
    rhs = stepSize / 2.0f * (forces + stepSize * dfdx * vel);
    result = lhs.inverse() * rhs;

    for(int i = 0; i < particleSystem->m_numParticles; i++){
        Vector3f_ newVel;
        Vector3f_ newPos;
        // fixed particle
        if(FVal[i * 3 + 2].x() == 1.0f){
            newVel = state[i * 2 + 1];
            newPos = state[i * 2];
        }
        newVel = state[i * 2 + 1] + eigenVecToVecmathVec(result.segment<3>(i * 3));
        newPos = state[i * 2] + newVel;
        newState.push_back(newPos);
        newState.push_back(newVel);
    }

    particleSystem->setState(newState);
}
#include "ClothSystemWithImplicitSolver.h"

ClothSystemWithImplicitSolver::ClothSystemWithImplicitSolver(int C_SIZE):ClothSystem(C_SIZE){

    condition = new Condition(C_SIZE, C_SIZE);
    cgsolver = new CGSolver(C_SIZE*C_SIZE, C_SIZE);

    dx = 1.0 / (condition->Nx - 1);
    dt = 1.0f;

    element_num = (condition->Nx - 1) * (condition->Ny - 1) * 2;
    element = new int[element_num * 3];   // element - a face contains two triangle
    element_duv = new Eigen::Matrix2f[element_num];
    element_dwu = new Eigen::Vector3f[element_num];
    element_dwv = new Eigen::Vector3f[element_num];

    int count = 0;
    Eigen::Vector2f duv10, duv20;
    // the number of square
    for (int j = 0; j < (condition->Ny - 1); j++)
    {
        for (int i = 0; i < (condition->Nx - 1); i++)
        {
            int index = j * condition->Nx + i;
            // top left triangle
            element[count * 6 + 0] = index;
            element[count * 6 + 1] = index + condition->Nx;
            element[count * 6 + 2] = index + 1;
            // lower right triangle
            element[count * 6 + 3] = index + condition->Nx;
            element[count * 6 + 4] = index + condition->Nx + 1;
            element[count * 6 + 5] = index + 1;

            // 目前所有三角形的uv都是一样的，但我还是把计算公式在这里
            // 2 --- 3
            // |  \  |
            // 0 --- 1
            //
            duv10 = Eigen::Vector2f(0, dx);
            duv20 = Eigen::Vector2f(dx, 0);

            float det = 1 / (duv10(0) * duv20(1) - duv10(1) * duv20(0));
            element_duv[count * 2] << duv20(1), -duv20(0),
                    -duv10(1), duv10(0);
            element_duv[count * 2] *= det;

            element_dwu[count * 2](0) = (duv10(1) - duv20(1)) * det;
            element_dwu[count * 2](1) = duv20(1) * det;
            element_dwu[count * 2](2) = -duv10(1) * det;

            element_dwv[count * 2](0) = (duv20(0) - duv10(0)) * det;
            element_dwv[count * 2](1) = -duv20(0) * det;
            element_dwv[count * 2](2) = duv10(0) * det;

            //
            duv10 = Eigen::Vector2f(dx, 0);
            duv20 = Eigen::Vector2f(dx, -dx);

            det = 1 / (duv10(0) * duv20(1) - duv10(1) * duv20(0));
            element_duv[count * 2 + 1] << duv20(1), -duv20(0),
                    -duv10(1), duv10(0);
            element_duv[count * 2 + 1] *= det;

            element_dwu[count * 2 + 1](0) = (duv10(1) - duv20(1)) * det;
            element_dwu[count * 2 + 1](1) = duv20(1) * det;
            element_dwu[count * 2 + 1](2) = -duv10(1) * det;

            element_dwv[count * 2 + 1](0) = (duv20(0) - duv10(0)) * det;
            element_dwv[count * 2 + 1](1) = -duv20(0) * det;
            element_dwv[count * 2 + 1](2) = duv10(0) * det;

            count += 1;
        }
    }
}


Eigen::Vector3f vecmathVec3fToEigenVec3f(Vector3f_ vec){
    Eigen::Vector3f v;
    v(0) = vec.x();
    v(1) = vec.y();
    v(2) = vec.z();
    return v;
}


Vector3f_ eigenVec3fToVecmathVec3f(Eigen::Vector3f vec){
    float x = vec(0);
    float y = vec(1);
    float z = vec(2);
    Vector3f_ v(x, y, z);
    return v;
}

vector<Vector3f_> ClothSystemWithImplicitSolver::evalF(vector<Vector3f_> state) {
    vector<Vector3f_> f;

    Eigen::Vector3f wu;
    Eigen::Vector3f wv;
    Eigen::Matrix2f duv_inv;

    Eigen::Vector3f p0, p1, p2, p3; // position
    Eigen::Vector3f vel0, vel1, vel2, vel3; // velocity
    Eigen::Vector3f dp10;
    Eigen::Vector3f dp20;
    int idx0, idx1, idx2, idx3;

    dfdx = Eigen::MatrixXf::Zero(condition->node_num * 3, condition->node_num * 3);
    dfdv = Eigen::MatrixXf::Zero(condition->node_num * 3, condition->node_num * 3);
    forces = Eigen::VectorXf::Zero(condition->node_num * 3);

    for (int i = 0; i < element_num; i++) {
        duv_inv = element_duv[i];

        idx0 = element[i * 3 + 0];
        idx1 = element[i * 3 + 1];
        idx2 = element[i * 3 + 2];

        p0 = vecmathVec3fToEigenVec3f(particles[idx0].xyz());
        p1 = vecmathVec3fToEigenVec3f(particles[idx1].xyz());
        p2 = vecmathVec3fToEigenVec3f(particles[idx2].xyz());
        dp10 = p1 - p0;
        dp20 = p2 - p0;

        wu = dp10 * duv_inv(0, 0) + dp20 * duv_inv(1, 0);
        wv = dp10 * duv_inv(0, 1) + dp20 * duv_inv(1, 1);

        vel0 = vecmathVec3fToEigenVec3f(state[2 * idx0 + 1]);
        vel1 = vecmathVec3fToEigenVec3f(state[2 * idx1 + 1]);
        vel2 = vecmathVec3fToEigenVec3f(state[2 * idx2 + 1]);

        condition->computeStretchForces(wu, wv, element_dwu[i], element_dwv[i], \
                                p0, p1, p2, vel0, vel1, vel2, idx0, idx1, idx2);
        condition->computeShearForces(wu, wv, element_dwu[i], element_dwv[i], \
                                p0, p1, p2, vel0, vel1, vel2, idx0, idx1, idx2);
        // a face have two triangles
        if (i % 2 == 0) {
            idx0 = element[i * 3 + 0];
            idx1 = element[i * 3 + 2];
            idx2 = element[i * 3 + 1];
            idx3 = element[i * 3 + 4];

            p0 = vecmathVec3fToEigenVec3f(particles[idx0].xyz());
            p1 = vecmathVec3fToEigenVec3f(particles[idx1].xyz());
            p2 = vecmathVec3fToEigenVec3f(particles[idx2].xyz());
            p3 = vecmathVec3fToEigenVec3f(particles[idx3].xyz());

            vel0 = vecmathVec3fToEigenVec3f(state[2 * idx0 + 1]);
            vel1 = vecmathVec3fToEigenVec3f(state[2 * idx1 + 1]);
            vel2 = vecmathVec3fToEigenVec3f(state[2 * idx2 + 1]);
            vel3 = vecmathVec3fToEigenVec3f(state[2 * idx3 + 1]);

            condition->computeBendingForce(p0, p1, p2, p3, vel0, vel1, vel2, vel3, idx0, idx1, idx2, idx3);
        }

    }

    for (int i = 0; i < m_numParticles; i++) {
        // f - (vel, F/M)
        Vector3f_ vel = state[2 * i + 1];
        f.push_back(vel);

        Vector3f_ f_Gravity = Vector3f_(0, mg, 0);    //Gravity
        Vector3f_ f_Drag = -1 * vDrag * state[2 * i + 1];    //Drag
        Vector3f_ v =  eigenVec3fToVecmathVec3f(forces.segment<3>(i * 3));
        Vector3f_ f_Net = v + f_Gravity + f_Drag;    //Net Force
        f.push_back(f_Net);

        if(particles[i].w() == 1.0f){
            f.push_back(Vector3f_(1, 0, 0));
        } else {
            f.push_back(Vector3f_(0, 0, 0));
        }
    }


    return f;

}
#ifndef CLOTHANIMATION_CLOTHSYSTEMWITHIMPLICITSOLVER_H
#define CLOTHANIMATION_CLOTHSYSTEMWITHIMPLICITSOLVER_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ClothSystem.h"
#include "Condition.h"
#include "CGSolver.h"

class ClothSystemWithImplicitSolver : public ClothSystem{
public:
    float dx;
    float dt;

    Condition* condition;
    CGSolver* cgsolver;

    int element_num; // face number
    int* element;  // a element - a face contains two triangles (six particles' indexes)
    Eigen::Matrix2f* element_duv;
    Eigen::Vector3f* element_dwu;
    Eigen::Vector3f* element_dwv;

    ClothSystemWithImplicitSolver(int C_SIZE);

    // stepSize - dx
    //Eigen::VectorXf computeVelChange(vector<Vector3f_> state, float stepSize);

    vector<Vector3f_> evalF(vector<Vector3f_> state);
};


#endif //CLOTHANIMATION_CLOTHSYSTEMWITHIMPLICITSOLVER_H

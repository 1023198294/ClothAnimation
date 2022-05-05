#ifndef CLOTHANIMATION_CGSOLVER_H
#define CLOTHANIMATION_CGSOLVER_H

#include <Eigen/Core>
#include <Eigen/Dense>

extern Eigen::MatrixXf lhs;
extern Eigen::VectorXf rhs;

class CGSolver {
public:
    int n;
    int nx;
    Eigen::VectorXf direction;
    Eigen::VectorXf residual;
    Eigen::VectorXf result;
    float dAd;
    float alpha_old;
    float alpha;
    float beta;

    CGSolver(int num,int nx0)
    {
        n = num;
        nx = nx0;
        lhs = Eigen::MatrixXf::Zero(n * 3,n * 3);
        rhs = Eigen::VectorXf::Zero(n * 3);
    }

    int colToIdx(int idx, int k0);

    Eigen::Vector3f Ax(int i);

    Eigen::Vector3f Ad(int i);

    bool solveStep();

    Eigen::VectorXf solve();
};


#endif //CLOTHANIMATION_CGSOLVER_H

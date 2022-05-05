#include "CGSolver.h"

Eigen::MatrixXf lhs;
Eigen::VectorXf rhs;

int CGSolver::colToIdx(int idx, int k0)
{
    int i0 = idx % nx;
    int j0 = idx / nx;
    switch (k0)
    {
        case 0:
            break;
        case 1:
            if (i0 > 0 && j0 > 0)idx = idx - nx - 1;
            else idx = -1;
            break;
        case 2:
            if (j0 > 0)idx = idx - nx;
            else idx = -1;
            break;
        case 3:
            if (i0 < nx - 1 && j0 > 0)idx = idx - nx + 1;
            else idx = -1;
            break;
        case 4:
            if (i0 > 0)idx = idx - 1;
            else idx = -1;
            break;
        case 5:
            if (i0 < nx - 1)idx = idx + 1;
            else idx = -1;
            break;
        case 6:
            if (i0 > 0 && j0 < nx - 1)idx = idx + nx - 1;
            else idx = -1;
            break;
        case 7:
            if (j0 < nx - 1)idx = idx + nx;
            else idx = -1;
            break;
        case 8:
            if (i0 < nx - 1 && j0 < nx - 1)idx = idx + nx + 1;
            else idx = -1;
            break;
        default:
            idx = -1;
            break;
    }
    return idx;
}

Eigen::Vector3f CGSolver::Ax(int i)
{
    Eigen::Vector3f term = Eigen::Vector3f::Zero();
    for (int j = 0; j < 9; j++)
    {
        int idx = colToIdx(i, j);
        if (idx != -1)
            term += lhs.block<3, 3>(i * 3, j * 3) * result.segment<3>(idx * 3);
    }
    return term;
}

Eigen::Vector3f CGSolver::Ad(int i)
{
    Eigen::Vector3f term = Eigen::Vector3f::Zero();
    for (int j = 0; j < 9; j++)
    {
        int idx = colToIdx(i, j);
        if (idx != -1)
            term += lhs.block<3, 3>(i * 3, j * 3) * direction.segment<3>(idx * 3);
    }
    return term;
}

bool CGSolver::solveStep()
{
    dAd = 1e-10;
    for (int i0 = 0; i0 < n; i0++){
        dAd += direction.segment<3>(i0 * 3).dot(Ad(i0));
    }
    alpha = residual.dot(residual) / dAd;
    if (alpha < 1e-10){
        return true;
    }
    result = result + alpha * direction;
    for (int i0 = 0; i0 < n; i0++){
        residual.segment<3>(i0 * 3) = residual.segment<3>(i0 * 3) - alpha * Ad(i0);
    }
    alpha_old = alpha;
    beta = residual.dot(residual) / ((alpha + 1e-10) * dAd);
    direction = residual + beta * direction;
    return false;
}

Eigen::VectorXf CGSolver::solve()
{
    result = Eigen::VectorXf::Zero(n * 3);
    direction = Eigen::VectorXf::Zero(n * 3);
    residual = Eigen::VectorXf::Zero(n * 3);
    for (int i = 0; i < n; i++)
    {
        residual.segment<3>(i * 3) = rhs.segment<3>(i * 3) - Ax(i);
        direction.segment<3>(i * 3) = residual.segment<3>(i * 3);
    }
    for (int k = 0; k < 10; k++)
    {
        if (solveStep())break;
    }
    return result;
}
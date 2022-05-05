#ifndef CLOTHANIMATION_CONDITION_H
#define CLOTHANIMATION_CONDITION_H

#include <Eigen/Core>
#include <Eigen/Dense>


extern Eigen::MatrixXf dfdx;
extern Eigen::MatrixXf dfdv;
extern Eigen::VectorXf forces;

class Condition {
public:

    int Nx;
    int Ny;
    int node_num;

    float a_stretch;
    float k_stretch;
    float damping_stretch;
    float a_shear;
    float k_shear;
    float damping_shear;
    float a_bend;
    float k_bend;
    float damping_bend;

    // related to calculation of force
    // derivative of force with respect to the particle's position
    Eigen::Matrix3f df0dx0, df0dx1, df0dx2, df0dx3;
    Eigen::Matrix3f df1dx0, df1dx1, df1dx2, df1dx3;
    Eigen::Matrix3f df2dx0, df2dx1, df2dx2, df2dx3;
    Eigen::Matrix3f df3dx0, df3dx1, df3dx2, df3dx3;
    // derivative of force with respect to the particle's velocity
    Eigen::Matrix3f df0dv0, df0dv1, df0dv2, df0dv3;
    Eigen::Matrix3f df1dv0, df1dv1, df1dv2, df1dv3;
    Eigen::Matrix3f df2dv0, df2dv1, df2dv2, df2dv3;
    Eigen::Matrix3f df3dv0, df3dv1, df3dv2, df3dv3;

    Eigen::Matrix3f identity;
    // related to bending force
    Eigen::Vector3f v10, v20, v23, v13, v12;  // edge
    Eigen::Vector3f e10, e20, e23, e13, e12;  // normalized edge
    float c00, c01, c02, c13, c11, c12; // cosine of the angles
    Eigen::Vector3f n0, n1;  // normalized triangle normals
    Eigen::Vector3f b00, b01, b02, b13, b11, b12; // the direction from a vertex of the triangle to the opposite side
    float d00, d01, d02, d13, d11, d12; // distance from vertex to opposite edge
    double theta, sinTheta, cosTheta, dThetadt; // dihedral angle and the derivative of theta with respect to the time
    Eigen::Matrix3f dn0dx0, dn0dx1, dn0dx2, dn0dx3, dn1dx0, dn1dx1, dn1dx2, dn1dx3; // derivatives of triangle normals
    Eigen::Vector3f dThetadx0, dThetadx1, dThetadx2, dThetadx3; // derivatives of triangle angle

    // derivatives of the perpendicular distances:
    Eigen::Vector3f dd00dx0, dd00dx1, dd00dx2, dd00dx3;
    Eigen::Vector3f dd01dx0, dd01dx1, dd01dx2, dd01dx3;
    Eigen::Vector3f dd02dx0, dd02dx1, dd02dx2, dd02dx3;
    Eigen::Vector3f dd11dx0, dd11dx1, dd11dx2, dd11dx3;
    Eigen::Vector3f dd12dx0, dd12dx1, dd12dx2, dd12dx3;
    Eigen::Vector3f dd13dx0, dd13dx1, dd13dx2, dd13dx3;

    // derivatives of the cosines:
    Eigen::Vector3f dc01dx0, dc01dx1, dc01dx2, dc01dx3, dc02dx0, dc02dx1, dc02dx2, dc02dx3;
    Eigen::Vector3f dc11dx0, dc11dx1, dc11dx2, dc11dx3, dc12dx0, dc12dx1, dc12dx2, dc12dx3;

    // second derivatives of theta with respect to the different vertex positions
    Eigen::Matrix3f d2Thetadx0dx0, d2Thetadx0dx1, d2Thetadx0dx2, d2Thetadx0dx3;
    Eigen::Matrix3f d2Thetadx1dx0, d2Thetadx1dx1, d2Thetadx1dx2, d2Thetadx1dx3;
    Eigen::Matrix3f d2Thetadx2dx0, d2Thetadx2dx1, d2Thetadx2dx2, d2Thetadx2dx3;
    Eigen::Matrix3f d2Thetadx3dx0, d2Thetadx3dx1, d2Thetadx3dx2, d2Thetadx3dx3;

    Condition(int nx, int ny){
        Nx = nx;
        Ny = ny;
        node_num = Nx * Ny;
        dfdx = Eigen::MatrixXf::Zero(node_num * 3, node_num * 3);
        dfdv = Eigen::MatrixXf::Zero(node_num * 3, node_num * 3);
        forces = Eigen::VectorXf::Zero(node_num * 3);

        a_stretch = 1.0f;
        k_stretch = 1.0f;
        damping_stretch = 2.0f;

        a_shear = 1.0f;
        k_shear = 1.0f;
        damping_shear = 2.0f;

        a_bend = 1.0f;
        k_bend = 1.0f;
        damping_bend = 2.0f;

        identity = Eigen::Matrix3f::Identity(3, 3);

    }

    void computeStretchForces(Eigen::Vector3f wu, Eigen::Vector3f wv, Eigen::Vector3f dwu, Eigen::Vector3f dwv, \
                              Eigen::Vector3f particlePos0, Eigen::Vector3f particlePos1, Eigen::Vector3f particlePos2, \
                              Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, \
                              int idx0, int idx1, int idx2);


    void computeShearForces(Eigen::Vector3f wu, Eigen::Vector3f wv, Eigen::Vector3f dwu, Eigen::Vector3f dwv, \
                            Eigen::Vector3f particlePos0, Eigen::Vector3f particlePos1, Eigen::Vector3f particlePos2, \
                            Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, \
                            int idx0, int idx1, int idx2);

    void computeBendingForce(Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, \
                             Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, \
                             int idx0, int idx1, int idx2, int idx3);
};


#endif //CLOTHANIMATION_CONDITION_H

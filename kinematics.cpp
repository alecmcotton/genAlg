#include "kinematics.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

Matrix4d dh_transform(double a, double alpha, double d, double theta) {
    Matrix4d T;
    T << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
         0,           sin(alpha),             cos(alpha),            d,
         0,           0,                      0,                     1;
    return T;
}

Matrix4d forward_kinematics(const VectorXd &theta, double L) {
    Matrix4d A1 = dh_transform(0,     -M_PI/2, 135, theta(0));
    Matrix4d A2 = dh_transform(135,    0,       0,  theta(1) - M_PI/2);
    Matrix4d A3 = dh_transform(38,    -M_PI/2,  0,  theta(2));
    Matrix4d A4 = dh_transform(0,      M_PI/2, 120, theta(3));
    Matrix4d A5 = dh_transform(0,     -M_PI/2,  0,  theta(4));
    Matrix4d A6 = dh_transform(L,      0,      70,  theta(5) + M_PI);
    return A1 * A2 * A3 * A4 * A5 * A6;
}

Vector3d rotation_error(const Matrix3d &R1, const Matrix3d &R2) {
    Matrix3d R_err = R1.transpose() * R2;
    AngleAxisd aa(R_err);
    return aa.axis() * aa.angle();
}

MatrixXd numerical_jacobian(const VectorXd &theta, double L) {
    double delta = 1e-6;
    Matrix4d T0 = forward_kinematics(theta, L);
    Vector3d p0 = T0.block<3,1>(0,3);
    Matrix3d R0 = T0.block<3,3>(0,0);

    MatrixXd J(6, 6);
    for (int i = 0; i < 6; ++i) {
        VectorXd perturbed = theta;
        perturbed(i) += delta;
        Matrix4d Ti = forward_kinematics(perturbed, L);
        Vector3d pi = Ti.block<3,1>(0,3);
        Matrix3d Ri = Ti.block<3,3>(0,0);
        Vector3d dp = (pi - p0) / delta;
        Vector3d drot = rotation_error(R0, Ri) / delta;
        J.block<3,1>(0,i) = dp;
        J.block<3,1>(3,i) = drot;
    }
    return J;
}

std::pair<VectorXd, std::vector<double>> inverse_kinematics_NR(
    const Matrix4d &T_target,
    VectorXd theta0,
    double L,
    int max_iters,
    double tol
) {
    VectorXd theta = theta0;
    std::vector<double> errors;
    double alpha = 0.1;

    for (int i = 0; i < max_iters; ++i) {
        Matrix4d T_curr = forward_kinematics(theta, L);
        Vector3d ep = T_target.block<3,1>(0,3) - T_curr.block<3,1>(0,3);
        Vector3d eo = rotation_error(T_curr.block<3,3>(0,0), T_target.block<3,3>(0,0));
        VectorXd e(6);
        e << ep, eo;
        errors.push_back(e.norm());

        if (e.norm() < tol)
            break;

        MatrixXd J = numerical_jacobian(theta, L);
        theta += alpha * J.completeOrthogonalDecomposition().solve(e);
    }
    return {theta, errors};
}



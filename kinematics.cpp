#include "kinematics.h"
#include <cmath>

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;

Matrix4d computeDHMatrix(const DHParam& param) {
    double ct = cos(param.theta);
    double st = sin(param.theta);
    double ca = cos(param.alpha);
    double sa = sin(param.alpha);

    Matrix4d T;
    T << ct, -st * ca,  st * sa, param.a * ct,
         st,  ct * ca, -ct * sa, param.a * st,
          0,       sa,       ca,       param.d,
          0,        0,        0,           1;
    return T;
}

Matrix4d computeForwardTransform(const std::vector<DHParam>& dhParams) {
    Matrix4d T = Matrix4d::Identity();
    for (const auto& param : dhParams) {
        T *= computeDHMatrix(param);
    }
    return T;
}

Eigen::VectorXd forwardKinematics(const std::vector<DHParam>& dhParams) {
    Eigen::Matrix4d T = computeForwardTransform(dhParams);
    Eigen::Vector3d position = T.block<3,1>(0, 3);
    Eigen::Matrix3d R = T.block<3,3>(0, 0);
    Eigen::Vector3d eulers = computeEulerAngles(R);

    Eigen::VectorXd result(6);
    result << position, eulers;
    return result;
}

Eigen::Vector3d computeEulerAngles(const Eigen::Matrix3d& R) {
    double alpha, beta, gamma;

    if (std::abs(R(2, 0)) < 1.0) {
        beta = std::asin(-R(2, 0));
        alpha = std::atan2(R(2, 1) / std::cos(beta), R(2, 2) / std::cos(beta));
        gamma = std::atan2(R(1, 0) / std::cos(beta), R(0, 0) / std::cos(beta));
    } else {
        beta = M_PI_2 * std::copysign(1.0, -R(2, 0));
        alpha = 0.0;
        gamma = std::atan2(-R(0, 1), R(1, 1));
    }

    return Eigen::Vector3d(alpha, beta, gamma);
}

double computeManipulability(const MatrixXd& jacobian) {
    MatrixXd JJt = jacobian * jacobian.transpose();
    return std::sqrt(JJt.determinant());
}

Eigen::Matrix3d rotationMatrix(const eulerAngles& eulers) {
	alpha = eulers[0];
	beta = eulers[1];
	gamma = eulers[2];

	Matrix3d Rx;
	Matrix3d Ry;
	Matrix3d Rz;

	Rx << 1, 0, 0, 
		0, cos(alpha), -sin(alpha), 
		0, sin(alpha), cos(alpha);
	Ry << cos(beta), 0, sin(beta), 
		0, 1, 0, 
		-sin(beta, 0, cos(beta);
	Rz << cos(gamma), -sin(gamma), 0, 
		sin(gamma), cos(gamma), 0, 
		0, 0, 1;
	R = Rz*Ry*Rx;
	
	return R;
}

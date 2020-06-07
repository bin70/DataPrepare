#pragma once
#include <common.hpp>

namespace loam {

// 把loam中的欧拉角形式的位姿(roll,pitch,yaw,x,y,z)转化为变换矩阵
inline Eigen::Matrix4d euler2matrix(Eigen::Matrix<double, 6, 1> &pose)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(pose[0], Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pose[1], Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pose[2], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Matrix3d R = Ry * Rx * Rz; //位姿转成全局矩阵的关键

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        m(i, j) = R(i, j); //旋转部分
    for (int i = 0; i < 3; ++i)
        m(i, 3) = pose[3 + i]; //平移部分
    return m;
}

} //end namespace loam
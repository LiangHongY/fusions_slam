/*
 * @Author: ylh 
 * @Date: 2024-04-20 10:50:05 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-01 12:34:44
 */
#ifndef MATH_H
#define MATH_H
#include <Eigen/Dense>
#include <iostream>

constexpr double PI = 3.14159265358979323846;

static Eigen::Matrix4d quaternionTrans2Mat4d(const Eigen::Quaterniond & q,const Eigen::Vector3d & t){
    Eigen::Matrix4d ans;
    ans.setIdentity();
    ans.block<3,3>(0,0) = q.toRotationMatrix();
    ans.block<3,1>(0,3) = t;
    return ans;
}

template< typename pointTypeT>
static bool planarCheck(const std::vector<pointTypeT>& points, Eigen::Vector4d &pabcd, float threhold){
    Eigen::Vector3d normalVector;
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    int pointsSize = points.size();
    A.resize(pointsSize,3);
    B.resize(pointsSize);
    B.setOnes();
    B = -1*B;
    for (int i = 0; i < pointsSize; i++)
    {
        A(i,0) = points[i].x;
        A(i,1) = points[i].y;
        A(i,2) = points[i].z;
    }

    normalVector = A.colPivHouseholderQr().solve(B);

    double normal = normalVector.norm();
    // normalVector.normalize();
    pabcd(0) = normalVector(0)/normal;
    pabcd(1) = normalVector(1)/normal;
    pabcd(2) = normalVector(2)/normal;
    pabcd(3) = 1.0/normal;

    for (int j = 0; j < pointsSize; j++)
    {
        if (fabs(pabcd(0) * points[j].x + pabcd(1) * points[j].y + pabcd(2) * points[j].z + pabcd(3)) > threhold)
        {
            return false;
        }
    }

    return true;

}
template<typename PointTypeT,typename T>
static PointTypeT transformPoint(PointTypeT point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t){
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = q*ep+t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}
/***************************************************************/
void rpy2q(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q);

void q2rpy(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy);

void q2R(const Eigen::Quaterniond& q_, Eigen::Matrix3d& R);

void R2q(const Eigen::Matrix3d& R, Eigen::Quaterniond& q);

void q2AxisAngle(const Eigen::Quaterniond& q, Eigen::AngleAxisd& aa);

void axisAngle2q(const Eigen::AngleAxisd& aa, Eigen::Quaterniond& q);

void rpy2R(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R);

void R2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy);

//将角度限制在[-PI,PI]
double normalizeRadian(double rad);

/*boreas 数据集中的roll,pitch,yaw转换为旋转矩阵*/
void roll2Rot(const double& roll, Eigen::Matrix3d& R);
void pitch2Rot(const double& pitch, Eigen::Matrix3d& R);
void yaw2Rot(const double& yaw, Eigen::Matrix3d& R);
void rpy2Rot(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R);
void rot2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy);
// boreas 数据集中的roll,pitch,yaw转换为旋转矩阵

#endif

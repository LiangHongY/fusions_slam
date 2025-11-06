/*
 * @Author: ylh 
 * @Date: 2024-04-07 23:05:25 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-05-05 14:59:34
 */

#ifndef IESKF_H
#define IESKF_H
#include <Eigen/Dense>
#include <glog/logging.h>
#include "fusion_slam/math/so3.h"
#include "fusion_slam/type/imu_type.h"
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/pose.h"
#include "fusion_slam/type/velocity.h"
#include "fusion_slam/math/math.h"
#include "fusion_slam/type/timestamp.h"
#include "fusion_slam/type/base_type.h"
#include <yaml-cpp/yaml.h>

struct StateX
{
    TimeStamp timestamp;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Eigen::Vector3d gravity;
    StateX(){
        timestamp.fromSec(0.0);
        rotation = Eigen::Quaterniond::Identity();
        position = Eigen::Vector3d::Zero();
        velocity = Eigen::Vector3d::Zero();
        bg = Eigen::Vector3d::Zero();
        ba = Eigen::Vector3d::Zero();
        gravity = Eigen::Vector3d::Zero();
    }
};

struct StateXP{
    TimeStamp timestamp;
    StateX st;
    Eigen::Matrix<double,18,18> P;
    Eigen::Matrix<double,18,18> P_;
};

const int NEAR_POINTS_NUM = 5;
class IESKF{
    private:
        StateX X;
        Eigen::Matrix<double,18,18> P;
        Eigen::Matrix<double,12,12> Q;
        Eigen::Matrix<double,18,18> P_;
        Eigen::Matrix<double,18,18> Q_; 
        int iterTimes;
        int imuHz;
        bool updateBaBg;
        Eigen::Matrix<double, 18, 1> dX;
     
        using lossType = triple<Eigen::Vector3d,Eigen::Vector3d,double>;
        KDTreeConstPtr globalMapKdtreePtr;
        PCLPointCloudPtr curCloudPtr;
        PCLPointCloudConstPtr localMapPtr;
        IKDTreePtr globalMapIkdtreePtr;
        bool calculateLidarZH(const StateX& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H);
        bool calculateLidarZH(const StateX& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H, std::vector<IKDTree::PointVector>& nearPoints);

    public:
        IESKF(const std::string& configPath);
        ~IESKF();
        using Ptr = std::shared_ptr<IESKF>;
        void predict(const ImuType& imuData);
        bool lidarObserve(const PointCloud& cloud, const KDTreeConstPtr& mapKdtreePtr, const PCLPointCloudConstPtr& mapPtr);
        bool lidarObserve(const PointCloud& cloud, const IKDTreePtr& mapIkdtreePtr, const PCLPointCloudConstPtr& mapPtr, std::vector<IKDTree::PointVector>& nearPoints);
        int rotationObserve(const Rot& rot, const Eigen::Vector3d& nosie);
        int positionObserve(const Pos& pos, const Eigen::Vector3d& nosie);
        int velocityObserve(const Velocity& vel, const Eigen::Vector3d& nosie);
        void updateAndReset();
        const StateX& getX();
        void setX(const StateX& x_in);
        Eigen::Matrix<double,18,1> getErrorStateX(const StateX &s1, const StateX &s2);
};
#endif

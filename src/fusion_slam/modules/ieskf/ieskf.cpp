/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:20:38 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:34:59
 */

#include "fusion_slam/modules/ieskf/ieskf.h"

IESKF::IESKF(const std::string& configPath){

        FLAGS_log_dir = "./data/logs";
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
        FLAGS_max_log_size = 10;
        google::InitGoogleLogging("IESKF");

        P.setIdentity();
        P(9, 9) = P(10, 10) = P(11, 11) = 0.0001;
        P(12, 12) = P(13, 13) = P(14, 14) = 0.001;
        P(15, 15) = P(16, 16) = P(17, 17) = 0.00001;

        P_ = Eigen::Matrix<double,18,18>::Identity()*1e-4;

        iterTimes = 10;
        double covGyroscope = 0.0001, covAcceleration = 0.01, covBiasAcceleration = 0.00001, covBiasGyroscope = 0.0001;
        updateBaBg = false;
        YAML::Node configNode;
        configNode = YAML::LoadFile(configPath);
        updateBaBg = configNode["ieskf"]["update_ba_bg"].as<bool>();
        covGyroscope = configNode["ieskf"]["cov_gyroscope"].as<double>();
        covAcceleration = configNode["ieskf"]["cov_acceleration"].as<double>();
        covBiasAcceleration = configNode["ieskf"]["cov_bias_acceleration"].as<double>();
        covBiasGyroscope = configNode["ieskf"]["cov_bias_gyroscope"].as<double>();
        imuHz = configNode["ieskf"]["imu_hz"].as<double>();

        Q.block<3, 3>(0, 0).diagonal() =
            Eigen::Vector3d{covGyroscope, covGyroscope, covGyroscope};
        Q.block<3, 3>(3, 3).diagonal() =
            Eigen::Vector3d{covAcceleration, covAcceleration, covAcceleration};
        Q.block<3, 3>(6, 6).diagonal() =
            Eigen::Vector3d{covBiasGyroscope, covBiasGyroscope, covBiasGyroscope};
        Q.block<3, 3>(9, 9).diagonal() =
            Eigen::Vector3d{covBiasAcceleration, covBiasAcceleration, covBiasAcceleration};
        Q_ = Eigen::Matrix<double,18,18>::Zero();
        Q_.diagonal() << covGyroscope, covGyroscope, covGyroscope,
                            0.0, 0.0, 0.0,
                            covAcceleration, covAcceleration, covAcceleration,
                            covBiasGyroscope, covBiasGyroscope, covBiasGyroscope,
                            covBiasAcceleration, covBiasAcceleration, covBiasAcceleration,
                            0.0, 0.0, 0.0;
        X.ba.setZero();
        X.bg.setZero();
        X.gravity.setZero();
        X.position.setZero();
        X.rotation.setIdentity();
        X.velocity.setZero();
        X.timestamp.fromSec(0.0);
        dX.setZero();
        LOG(INFO) <<"\033[1;32m"<< "ieskf: " << "\033[0m" << std::endl;
        std::cout << "      update_ba_bg            : " << updateBaBg << std::endl;
        std::cout << "      cov_gyroscope           : " << covGyroscope << std::endl;
        std::cout << "      cov_acceleration        : " << covAcceleration << std::endl;
        std::cout << "      cov_bias_acceleration   : " << covBiasAcceleration << std::endl;
        std::cout << "      cov_bias_gyroscope      : " << covBiasGyroscope << std::endl;
        std::cout << "      imu_hz                  : " << imuHz << std::endl;
    }

IESKF::~IESKF() {}
void IESKF::predict(const ImuType& imuData){
    ImuType imu = imuData;
    if(imu.timeStamp.sec() < X.timestamp.sec()) return;
    double dt = imu.timeStamp.sec() - X.timestamp.sec();
    if (dt > 5.0/imuHz || dt < 0) {
        LOG(INFO) << "skip this imu because dt " << dt << std::endl;
        X.timestamp.fromSec(imu.timeStamp.sec());
        return;
    }
    imu.acceleration -= X.ba;
    imu.gyroscope -= X.bg;
    auto rotation = X.rotation.toRotationMatrix();

    X.rotation = Eigen::Quaterniond(rotation * so3Exp((imu.gyroscope) * dt));
    X.rotation.normalize();
    X.position += X.velocity * dt;
    X.velocity += (rotation * (imu.acceleration) + X.gravity) * dt;

    Eigen::Matrix<double, 18, 18> Fx;
    Eigen::Matrix<double, 18, 12> Fw;
    Fw.setZero();
    Fx.setIdentity();
    Fx.block<3, 3>(0, 0) = so3Exp(-1 * imu.gyroscope * dt);
    Fx.block<3, 3>(0, 9) = -1 * J_right(-imu.gyroscope * dt) * dt;
    Fx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 0) = rotation * skewSymmetric(imu.acceleration) * dt * (-1);
    Fx.block<3, 3>(6, 12) = rotation * dt * (-1);
    Fx.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

    Fw.block<3, 3>(0, 0) = -1 * J_right(-imu.gyroscope * dt) * dt;
    Fw.block<3, 3>(6, 3) = -1 * rotation * dt;
    Fw.block<3, 3>(9, 6) = Fw.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;
    P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose();
    P_ = Fx*P_*Fx.transpose() + Q_;
    X.timestamp.fromSec(imu.timeStamp.sec());
}

bool IESKF::lidarObserve(const PointCloud& cloud, const KDTreeConstPtr& mapKdtreePtr, const PCLPointCloudConstPtr& mapPtr) {
    if(cloud.timeStamp.sec() < X.timestamp.sec()) return false;
    globalMapKdtreePtr = mapKdtreePtr;
    curCloudPtr = cloud.cloudPtr;
    localMapPtr = mapPtr;

    auto curX = X;
    Eigen::MatrixXd K;
    Eigen::MatrixXd curH;
    Eigen::Matrix<double, 18, 18> curP;
    bool converge = true;
    for (int i = 0; i < iterTimes; i++) {
        Eigen::Matrix<double, 18, 1> errorState = getErrorStateX(curX, X);
        Eigen::Matrix<double, 18, 18> J_inv;
        J_inv.setIdentity();
        J_inv.block<3, 3>(0, 0) = J_right(errorState.block<3, 1>(0, 0));
        curP = J_inv * P * J_inv.transpose();
        Eigen::MatrixXd curZ;
        calculateLidarZH(curX, curZ, curH);
        Eigen::MatrixXd curHT = curH.transpose();
        K = (curHT * curH + (curP / 0.001).inverse()).inverse() * curHT;
        Eigen::MatrixXd left = -1 * K * curZ;
        Eigen::MatrixXd right =
            -1 * (Eigen::Matrix<double, 18, 18>::Identity() - K * curH) * J_inv * errorState;
        Eigen::MatrixXd updateX = left + right;

        converge = true;
        for (int idx = 0; idx < 18; idx++) {
            if (updateX(idx, 0) > 0.001) {
                converge = false;
                break;
            }
        }
        curX.rotation = Eigen::Quaterniond(curX.rotation.toRotationMatrix() * so3Exp(updateX.block<3, 1>(0, 0)));
        curX.rotation.normalize();
        curX.position = curX.position + updateX.block<3, 1>(3, 0);
        curX.velocity = curX.velocity + updateX.block<3, 1>(6, 0);
        curX.bg = curX.bg + updateX.block<3, 1>(9, 0);
        curX.ba = curX.ba + updateX.block<3, 1>(12, 0);
        curX.gravity = curX.gravity + updateX.block<3, 1>(15, 0);
        if (converge) {
            break;
        }
    }
    X = curX;
    P = (Eigen::Matrix<double, 18, 18>::Identity() - K * curH) * curP;
    X.timestamp.fromSec(cloud.timeStamp.sec());
    return converge;
}

bool IESKF::lidarObserve(const PointCloud& cloud, const IKDTreePtr& mapIkdtreePtr, const PCLPointCloudConstPtr& mapPtr, std::vector<IKDTree::PointVector>& nearPoints) {
    if(cloud.timeStamp.sec() < X.timestamp.sec()) return false;
    globalMapIkdtreePtr = mapIkdtreePtr;
    curCloudPtr = cloud.cloudPtr;
    localMapPtr = mapPtr;

    auto curX = X;
    Eigen::MatrixXd K;
    Eigen::MatrixXd curH;
    Eigen::Matrix<double, 18, 18> curP;
    bool converge = true;
    for (int i = 0; i < iterTimes; i++) {
        Eigen::Matrix<double, 18, 1> errorState = getErrorStateX(curX, X);
        Eigen::Matrix<double, 18, 18> J_inv;
        J_inv.setIdentity();
        J_inv.block<3, 3>(0, 0) = J_right(errorState.block<3, 1>(0, 0));
        curP = J_inv * P * J_inv.transpose();
        Eigen::MatrixXd curZ;
        calculateLidarZH(curX, curZ, curH, nearPoints);
        Eigen::MatrixXd curHT = curH.transpose();
        K = (curHT * curH + (curP / 0.001).inverse()).inverse() * curHT;
        Eigen::MatrixXd left = -1 * K * curZ;
        Eigen::MatrixXd right =
            -1 * (Eigen::Matrix<double, 18, 18>::Identity() - K * curH) * J_inv * errorState;
        Eigen::MatrixXd updateX = left + right;

        converge = true;
        for (int idx = 0; idx < 18; idx++) {
            if (updateX(idx, 0) > 0.001) {
                converge = false;
                break;
            }
        }
        curX.rotation = Eigen::Quaterniond(curX.rotation.toRotationMatrix() * so3Exp(updateX.block<3, 1>(0, 0)));
        curX.rotation.normalize();
        curX.position = curX.position + updateX.block<3, 1>(3, 0);
        curX.velocity = curX.velocity + updateX.block<3, 1>(6, 0);
        curX.bg = curX.bg + updateX.block<3, 1>(9, 0);
        curX.ba = curX.ba + updateX.block<3, 1>(12, 0);
        curX.gravity = curX.gravity + updateX.block<3, 1>(15, 0);
        if (converge) {
            break;
        }
    }
    X = curX;
    P = (Eigen::Matrix<double, 18, 18>::Identity() - K * curH) * curP;
    X.timestamp.fromSec(cloud.timeStamp.sec());
    return converge;
}

bool IESKF::calculateLidarZH(const StateX& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H, std::vector<IKDTree::PointVector>& nearPoints){
    // LOG(INFO) << "nearPoints size " << nearPoints.size() ;
    std::vector<lossType> lossV;
    lossV.resize(curCloudPtr->size());
    std::vector<bool> isEffectPoint(curCloudPtr->size(),false);
    std::vector<lossType> lossReal;
    int  vaildPointsNum = 0;
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (size_t  i = 0; i < curCloudPtr->size(); i++)
    {
        PointType imuPoint = curCloudPtr->points[i];
        PointType globalPoint;
        globalPoint = transformPoint(imuPoint, state.rotation, state.position);
        auto& pointsNear = nearPoints[i];
        std::vector<float> pointSearchSqDis(NEAR_POINTS_NUM);
        globalMapIkdtreePtr->Nearest_Search(globalPoint, NEAR_POINTS_NUM, pointsNear, pointSearchSqDis);
        if (pointsNear.size()<NEAR_POINTS_NUM || pointSearchSqDis[NEAR_POINTS_NUM-1]>5)
        {
            continue;
        }
        std::vector<PointType> planarPoints;
        for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
        {
            PointType point;
            point = pointsNear[ni];
            planarPoints.emplace_back(point);
        }
        Eigen::Vector4d pabcd;
        if (planarCheck(planarPoints,pabcd,0.1))    
        {
            double pd = globalPoint.x*pabcd(0)+globalPoint.y*pabcd(1)+globalPoint.z*pabcd(2)+pabcd(3);
            lossType loss;
            loss.thrid = pd;
            loss.first = {imuPoint.x, imuPoint.y, imuPoint.z};
            loss.second = pabcd.block<3,1>(0,0);
            if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
            double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
            if(s > 0.9 ){
                vaildPointsNum++;
                lossV[i] = loss;
                isEffectPoint[i] = true;
            }
        }

    }
    for (size_t i = 0; i <curCloudPtr->size(); i++)
    {
        if(isEffectPoint[i]) lossReal.emplace_back(lossV[i]);
    }
    vaildPointsNum = lossReal.size();
    // LOG(INFO) << "vaildPointsNum " << vaildPointsNum;
    H = Eigen::MatrixXd::Zero(vaildPointsNum, 18); 
    Z.resize(vaildPointsNum,1);
    for (int vi = 0; vi < vaildPointsNum; vi++)
    {
        Eigen::Vector3d dr = -1*lossReal[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(lossReal[vi].first);
        H.block<1,3>(vi,0) = dr.transpose();
        H.block<1,3>(vi,3) = lossReal[vi].second.transpose();
        Z(vi,0) = lossReal[vi].thrid;
    }
    return true;
}

bool IESKF::calculateLidarZH(const StateX& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H){
    std::vector<lossType> lossV;
    lossV.resize(curCloudPtr->size());
    std::vector<bool> isEffectPoint(curCloudPtr->size(),false);
    std::vector<lossType> lossReal;
    int  vaildPointsNum = 0;
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (size_t  i = 0; i < curCloudPtr->size(); i++)
    {
        PointType imuPoint = curCloudPtr->points[i];
        PointType globalPoint;
        globalPoint = transformPoint(imuPoint, state.rotation, state.position);
        std::vector<int> pointIds;
        std::vector<float> distance;
        globalMapKdtreePtr->nearestKSearch(globalPoint, NEAR_POINTS_NUM, pointIds, distance);
        if (distance.size()<NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM-1]>5)
        {
            continue;
        }
        std::vector<PointType> planarPoints;
        for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
        {
            planarPoints.emplace_back(localMapPtr->at(pointIds[ni]));
        }
        Eigen::Vector4d pabcd;
        if (planarCheck(planarPoints,pabcd,0.1))    
        {
            double pd = globalPoint.x*pabcd(0)+globalPoint.y*pabcd(1)+globalPoint.z*pabcd(2)+pabcd(3);
            lossType loss;
            loss.thrid = pd;
            loss.first = {imuPoint.x, imuPoint.y, imuPoint.z};
            loss.second = pabcd.block<3,1>(0,0);
            if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
            double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
            if(s > 0.9 ){
                vaildPointsNum++;
                lossV[i] = loss;
                isEffectPoint[i] = true;
            }
        }

    }
    for (size_t i = 0; i <curCloudPtr->size(); i++)
    {
        if(isEffectPoint[i]) lossReal.emplace_back(lossV[i]);
    }
    vaildPointsNum = lossReal.size();
    H = Eigen::MatrixXd::Zero(vaildPointsNum, 18); 
    Z.resize(vaildPointsNum,1);
    for (int vi = 0; vi < vaildPointsNum; vi++)
    {
        Eigen::Vector3d dr = -1*lossReal[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(lossReal[vi].first);
        H.block<1,3>(vi,0) = dr.transpose();
        H.block<1,3>(vi,3) = lossReal[vi].second.transpose();
        Z(vi,0) = lossReal[vi].thrid;
    }
    return true;
}

Eigen::Matrix<double, 18, 1> IESKF::getErrorStateX(const StateX &s1, const StateX &s2) {
    Eigen::Matrix<double, 18, 1> es;
    es.setZero();
    es.block<3, 1>(0, 0) =
        SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
    es.block<3, 1>(3, 0) = s1.position - s2.position;
    es.block<3, 1>(6, 0) = s1.velocity - s2.velocity;
    es.block<3, 1>(9, 0) = s1.bg - s2.bg;
    es.block<3, 1>(12, 0) = s1.ba - s2.ba;
    es.block<3, 1>(15, 0) = s1.gravity - s2.gravity;
    return es;
}

const StateX& IESKF::getX() { return X; }
void IESKF::setX(const StateX &x_in) { X = x_in; }

void IESKF::updateAndReset(){
    X.rotation = Eigen::Quaterniond(X.rotation.toRotationMatrix() * so3Exp(dX.template block<3, 1>(0, 0)));
    X.rotation.normalize();
    X.position += dX.template block<3, 1>(3, 0);
    X.velocity += dX.template block<3, 1>(6, 0);
    if(updateBaBg){
        X.bg += dX.template block<3, 1>(9, 0);
        X.ba += dX.template block<3, 1>(12, 0);
    }
    X.gravity += dX.template block<3, 1>(15, 0);
    Eigen::Matrix<double,18,18> J = Eigen::Matrix<double,18,18>::Identity();
    J.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - 0.5*skewSymmetric(dX.template block<3, 1>(0, 0));
    P_ = J*P_*J.transpose();
    dX.setZero();
}

int IESKF::rotationObserve(const Rot& rot, const Eigen::Vector3d& nosie){
    if(rot.timeStamp.sec() < X.timestamp.sec()) return -1;
    Eigen::Matrix3d rotNosie = Eigen::Matrix3d::Zero();
    rotNosie.diagonal() = nosie;
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 18, 3> K = P_ * H.transpose() * (H * P_ * H.transpose() + rotNosie).inverse();
    Eigen::Vector3d dRot = Eigen::Vector3d::Zero();
    dRot = SO3Log(X.rotation.toRotationMatrix().inverse() * rot.rotation.toRotationMatrix());
    dX = K * dRot;
    P_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P_;
    updateAndReset();
    X.timestamp.fromSec(rot.timeStamp.sec());
    return 0;
}
int IESKF::positionObserve(const Pos& pos, const Eigen::Vector3d& nosie){
    if(pos.timeStamp.sec() < X.timestamp.sec()) return -1;
    Eigen::Matrix3d posNosie = Eigen::Matrix3d::Zero();
    posNosie.diagonal() = nosie;
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 18, 3> K = P_ * H.transpose() * (H * P_ * H.transpose() + posNosie).inverse();
    Eigen::Vector3d dPos = Eigen::Vector3d::Zero();
    dPos = pos.position - X.position;
    dX = K * dPos;
    P_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P_;
    updateAndReset();
    X.timestamp.fromSec(pos.timeStamp.sec());
    return 0;
}
int IESKF::velocityObserve(const Velocity& vel, const Eigen::Vector3d& nosie){
    if(vel.timeStamp.sec() < X.timestamp.sec()) return -1;
    Eigen::Matrix3d velNosie = Eigen::Matrix3d::Zero();
    velNosie.diagonal() = nosie;
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 18, 3> K = P_ * H.transpose() * (H * P_ * H.transpose() + velNosie).inverse();
    Eigen::Vector3d dVel = Eigen::Vector3d::Zero();
    dVel = vel.velocity - X.velocity;
    dX = K * dVel;
    P_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P_;
    updateAndReset();
    X.timestamp.fromSec(vel.timeStamp.sec());
    return 0;
}

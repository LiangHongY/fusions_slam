/*
 * @Author: ylh 
 * @Date: 2024-09-13 15:54:07 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-09-13 22:33:14
 */

#include "fusion_slam/modules/location/location.h"

Location::Location(const std::string& configPath){
    inited = false;
    initUseRtk = false;
    locIeskfPtr = std::make_shared<IESKF>(configPath);
    locImuStaticInitPtr = std::make_shared<ImuStaticInit>(configPath);
    locMapEnginePtr = std::make_shared<MapEngine>(configPath);
    locPropagatePtr = std::make_shared<Propagate>(configPath);
    pcdPtrFilterResult = pcl::make_shared<PCLPointCloud>();
    imuInitDuration = 0.1;
    YAML::Node configNode;
    float leafSize = 0.2;
    imuAccScale = 1.0;
    configNode = YAML::LoadFile(configPath);
    initUseRtk = configNode["wrapper"]["use_rtk"].as<bool>();
    leafSize = configNode["location"]["leaf_size"].as<float>();
    imuInitDuration = configNode["location"]["imu_init_duration"].as<double>();
    pcdVoxelFilter.setLeafSize(leafSize, leafSize, leafSize);
    LOG(INFO) <<"\033[1;32m"<< "location: " << "\033[0m" << std::endl;
    std::cout << "      leaf_size           : " << leafSize << std::endl;
    std::cout << "      imu_init_duration   : " << imuInitDuration << std::endl;

}

Location::~Location(){}

int Location::addImu(const ImuType& imu){
    imus.emplace_back(imu);
    return 0;
}

int Location::addLidar(const PointCloud& cloud){
    clouds.emplace_back(cloud);
    return 0;
}

int Location::addRtk(const Pose& pose){
    rtkPoses.emplace_back(pose);
    return 0;
}

int Location::addVelocity(const Velocity& vel){
    vels.emplace_back(vel);
    return 0;
}

bool Location::run(){
    MeasureGroupAdd msg;
    if(syncMeasureGroupAdd(msg)){
        if(!inited){
            init(msg);
            static bool loadMapSucceed = false;
            if(!loadMapSucceed){
                auto ret = locMapEnginePtr->loadMap();
                if(ret == 0) loadMapSucceed = true;
            }
            sleep(1);
            return false;
        }
        auto t0 = GetCurrentTime::nowUs();
        locPropagatePtr->run(msg, locIeskfPtr);
        auto t1 = GetCurrentTime::nowUs();
        pcdVoxelFilter.setInputCloud(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr);
        pcdVoxelFilter.filter(*pcdPtrFilterResult);
        pcdPtrFilterResult->width = pcdPtrFilterResult->points.size();
        pcdPtrFilterResult->height = 1;
        pcdPtrFilterResult->is_dense = true;
        auto t2 = GetCurrentTime::nowUs();
        msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr = pcdPtrFilterResult;
        auto t3 = GetCurrentTime::nowUs();
        locIeskfPtr->lidarObserve(msg.mapDatas[msg.lidarBeginTime].cloud, locMapEnginePtr->readKDtree(), locMapEnginePtr->getMap());
        auto t4 = GetCurrentTime::nowUs();
        return true;
    }
    return false;
}

int Location::init(MeasureGroupAdd &msg){
    static int imuCnt = 0;
    static double startTs = 0.0;
    static bool firstImu = true;
    bool hasRtk = false;
    Pose initRtkPose;
    ImuType tmpImu;
    for(auto &data : msg.mapDatas){
        if(data.second.type == 0){
            if(firstImu){
                startTs = data.second.imu.timeStamp.sec();
                firstImu = false;
            }
            tmpImu = data.second.imu;
            locImuStaticInitPtr->addImu(data.second.imu);
        }
        if(data.second.type == 2){
            hasRtk = true;
            initRtkPose = data.second.rtkPose;
        }
    }
    if(tmpImu.timeStamp.sec() - startTs > imuInitDuration){
        if(initUseRtk){
            if(!hasRtk) return 0;
        }
        Eigen::Vector3d ba = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d bg = Eigen::Vector3d::Zero(); 
        double scale = 1.0;
        Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
        locImuStaticInitPtr->getStaticalImuParams(ba, bg, gravity, scale);
        auto state = locIeskfPtr->getX();
        state.timestamp.fromSec(tmpImu.timeStamp.sec());
        state.ba = ba;
        state.bg = bg;
        state.gravity = gravity;
        state.velocity = Eigen::Vector3d::Zero();
        if(initUseRtk){
            state.position = initRtkPose.position;
            state.rotation = initRtkPose.rotation;
            state.timestamp.fromSec(initRtkPose.timeStamp.sec());
        }
        locIeskfPtr->setX(state);
        inited = true;
        locPropagatePtr->imuAccScale = scale;
        locPropagatePtr->lastPcdEndTime = tmpImu.timeStamp.nsec();
        locPropagatePtr->lastImu = tmpImu;
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        state = locIeskfPtr->getX();
        q2rpy(state.rotation, rpy);
        LOG(INFO) << " imu init ba " << state.ba.transpose() << " bg " << state.bg.transpose() << " gravity " << state.gravity.transpose() <<" imuAccScale "<< scale << std::endl; 
        LOG(INFO)<< " init pos " << state.timestamp.sec() << " " << state.position.transpose() << " " << rpy.transpose()*180/M_PI << " vel " << state.velocity.transpose() 
        << " bg " << state.bg.transpose() << " ba " << state.ba.transpose() << " gravity " << state.gravity.transpose() << std::endl;
    }
    return 0;
}

bool Location::syncMeasureGroupAdd(MeasureGroupAdd& msg){
    auto t0 = GetCurrentTime::nowUs();
    msg.mapDatas.clear();
    if(imus.empty() || clouds.empty()){
        return false;
    }
    msg.lidarBeginTime = clouds.front().timeStamp.nsec();
    msg.lidarEndTime = clouds.front().cloudPtr->back().offset_time + msg.lidarBeginTime;
    if(imus.front().timeStamp.nsec() > msg.lidarEndTime){
        clouds.pop_front();
        return false;
    }
    DataUnit tmpCloud, tmpImu, tmpRtkPose;
    int cnt = 0;
    while(!imus.empty()){
        if(imus.front().timeStamp.nsec() < msg.lidarBeginTime ) {
            imus.pop_front();
            continue;
        }
        if(imus.front().timeStamp.nsec() > msg.lidarEndTime) {    
            break;         
        }
        tmpImu.type = 0;
        tmpImu.imu = imus.front();
        msg.mapDatas.emplace(imus.front().timeStamp.nsec(), tmpImu);
        imus.pop_front();
        ++cnt;
    }
    if(cnt < 5) return false;

    tmpCloud.type = 1;
    tmpCloud.cloud = clouds.front();
    msg.mapDatas.emplace(clouds.front().timeStamp.nsec(), tmpCloud);
    clouds.pop_front();

    while(!rtkPoses.empty()){
        if(rtkPoses.front().timeStamp.nsec() < msg.lidarBeginTime) {
            rtkPoses.pop_front();
            continue;
        }
        if(rtkPoses.front().timeStamp.nsec() > msg.lidarEndTime) {    
            break;         
        }
        tmpRtkPose.type = 2;
        tmpRtkPose.rtkPose = rtkPoses.front();
        if(msg.mapDatas.find(tmpRtkPose.rtkPose.timeStamp.nsec()) != msg.mapDatas.end()){
            while(1){
                tmpRtkPose.rtkPose.timeStamp.fromNsec(rtkPoses.front().timeStamp.nsec()+1);
                if(msg.mapDatas.find(tmpRtkPose.rtkPose.timeStamp.nsec()) == msg.mapDatas.end()) break;
            }
        }
        msg.mapDatas.emplace(tmpRtkPose.rtkPose.timeStamp.nsec(),tmpRtkPose);
        
        rtkPoses.pop_front();
    }
    auto t1 = GetCurrentTime::nowUs();
    // LOG(INFO) << std::fixed<< "syncMeasureGroupAdd cost " << (t1-t0)*1e-6 << std::endl;
    // LOG(INFO) <<std::fixed << " syncMeasureGroupAdd lidarBeginTime: " << msg.lidarBeginTime*1e-9 << " " << (tmpImu.imu.timeStamp.nsec() - msg.lidarBeginTime)*1e-9 << " lidarEndTime: " << msg.lidarEndTime*1e-9 << " " << (msg.lidarEndTime -tmpImu.imu.timeStamp.nsec())*1e-9 << " imu ts " << tmpImu.imu.timeStamp.sec() << std::endl;
    // LOG(INFO) << std::fixed << "lbt " << msg.lidarBeginTime << " let: " << msg.lidarEndTime << " mapDates size " << msg.mapDatas.size() << " " << cnt <<std::endl;
    
    return true;
}

const PCLPointCloud& Location::readCurrentPointCloud(){
    return *pcdPtrFilterResult;
}

const PCLPointCloud& Location::readCurrentLocalMap(){
    return *locMapEnginePtr->getMap();
}

StateX Location::readState(){
    return locIeskfPtr->getX();
}

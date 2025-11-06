/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:27:15 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 23:36:44
 */
#include "fusion_slam/modules/front/front.h"

Front::Front(const std::string &configPath){
    inited = false;
    initUseRtk = false;
    imuInitDuration = 1.0;
    ieskfPtr = std::make_shared<IESKF>(configPath);
    propagatePtr = std::make_shared<Propagate>(configPath);
    mapPtr = std::make_shared<Map>(configPath);
    imuStaticInitPtr = std::make_shared<ImuStaticInit>(configPath);
    imuAccScale = 1.0;
    pcdPtrFilterResult = pcl::make_shared<PCLPointCloud>();
    float leafSize = 0.2;
    YAML::Node configNode;
    configNode = YAML::LoadFile(configPath);
    initUseRtk = configNode["wrapper"]["use_rtk"].as<bool>();
    leafSize = configNode["front"]["leaf_size"].as<float>();
    imuInitDuration = configNode["front"]["imu_init_duration"].as<double>();
    pcdVoxelFilter.setLeafSize(leafSize, leafSize, leafSize);
    LOG(INFO) <<"\033[1;32m"<< "front: " << "\033[0m" << std::endl;
    std::cout << "      leaf_size           : " << leafSize << std::endl;
    std::cout << "      imu_init_duration   : " << imuInitDuration << std::endl;
}

Front::~Front(){}

int Front::addImu(const ImuType& imu){
    imus.emplace_back(imu);
    return 0;
}

int Front::addLidar(const PointCloud& cloud){
    clouds.emplace_back(cloud);
    return 0;
}

int Front::addRtk(const Pose& pose){
    rtkPoses.emplace_back(pose);
    return 0;
}

bool Front::run(){
    MeasureGroupAdd msg;
    if(syncMeasureGroupAdd(msg)){
        if(!inited){
            init(msg);
            mapPtr->reset();
            auto state = ieskfPtr->getX();
            auto pos = state.position;
            auto rot = state.rotation;
            // mapPtr->addPcdAndUpdateLocalMap(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr, rot, pos);
            nearstPoints.resize(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr->points.size());
            mapPtr->addPcdAndUpdateLocalMap(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr, state.rotation, state.position, nearstPoints);
            return false;
        }
        // LOG(INFO) << std::fixed << " lidarBeginTime: " << double(msg.lidarBeginTime*1e-9);
        auto t0 = GetCurrentTime::nowUs();
        propagatePtr->run(msg, ieskfPtr);
        auto t1 = GetCurrentTime::nowUs();
        pcdVoxelFilter.setInputCloud(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr);
        pcdVoxelFilter.filter(*pcdPtrFilterResult);

        pcdPtrFilterResult->width = pcdPtrFilterResult->points.size();
        pcdPtrFilterResult->height = 1;
        pcdPtrFilterResult->is_dense = true;

        auto t2 = GetCurrentTime::nowUs();
        msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr = pcdPtrFilterResult;
        auto t3 = GetCurrentTime::nowUs();
        nearstPoints.resize(pcdPtrFilterResult->points.size());
        // ieskfPtr->lidarObserve(msg.mapDatas[msg.lidarBeginTime].cloud, mapPtr->readKDtree(), mapPtr->getLocalMap());
        ieskfPtr->lidarObserve(msg.mapDatas[msg.lidarBeginTime].cloud, mapPtr->readIkdtree(), mapPtr->getLocalMap(), nearstPoints);
        auto t4 = GetCurrentTime::nowUs();
        if(0) // If you need to see map point, change to "if(1)"
        {
            PCLPointCloudPtr featsFromMap = pcl::make_shared<PCLPointCloud>();
            auto ikdtree = mapPtr->readIkdtree();
            IKDTree::PointVector ().swap(ikdtree->PCL_Storage);
            ikdtree->flatten(ikdtree->Root_Node, ikdtree->PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree->PCL_Storage;
            // LOG(INFO) << "featsFromMap->points.size(): " << featsFromMap->points.size() << endl;
        }
        auto state = ieskfPtr->getX();
        auto t5 = GetCurrentTime::nowUs();
        // mapPtr->addPcdAndUpdateLocalMap(pcdPtrFilterResult, state.rotation, state.position);
        mapPtr->addPcdAndUpdateLocalMap(msg.mapDatas[msg.lidarBeginTime].cloud.cloudPtr, state.rotation, state.position, nearstPoints);
        auto t6 = GetCurrentTime::nowUs();
        // LOG(INFO) << "propagate: " << (t1 - t0)*0.000001  << " lidar observe: " << (t4 - t3)*0.000001 <<  " update map: " << (t6 - t5)*0.000001 << std::endl;
        return true;
    }
    return false;
}

int Front::init(MeasureGroupAdd &msg){
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
            imuStaticInitPtr->addImu(data.second.imu);
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
        imuStaticInitPtr->getStaticalImuParams(ba, bg, gravity, scale);
        auto state = ieskfPtr->getX();
        state.timestamp.fromSec(tmpImu.timeStamp.sec());
        state.ba = ba;
        state.bg = bg;
        state.gravity = gravity;
        state.velocity = Eigen::Vector3d::Zero(); // todo: car in moving, set velocity by other sensor
        if(initUseRtk){
            state.position = initRtkPose.position;
            state.rotation = initRtkPose.rotation;
            state.timestamp.fromSec(initRtkPose.timeStamp.sec());
        }

        ieskfPtr->setX(state);
        inited = true;
        propagatePtr->imuAccScale = scale;
        propagatePtr->lastPcdEndTime = tmpImu.timeStamp.nsec();
        propagatePtr->lastImu = tmpImu;
        
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        state = ieskfPtr->getX();
        q2rpy(state.rotation, rpy);

        LOG(INFO) << " imu init ba " << state.ba.transpose() << " bg " << state.bg.transpose() << " gravity " << state.gravity.transpose() <<" imuAccScale "<< scale << std::endl; 
        LOG(INFO)<< " init pos " << state.timestamp.sec() << " " << state.position.transpose() << " " << rpy.transpose()*180/M_PI << " vel " << state.velocity.transpose() 
        << " bg " << state.bg.transpose() << " ba " << state.ba.transpose() << " gravity " << state.gravity.transpose() << std::endl;
    }
    return 0;
}

bool Front::syncMeasureGroupAdd(MeasureGroupAdd& msg){
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

const PCLPointCloud& Front::readCurrentPointCloud(){
    return *pcdPtrFilterResult;
}

const PCLPointCloud& Front::readCurrentLocalMap(){
    return *mapPtr->getLocalMap();
}

StateX Front::readState(){
    return ieskfPtr->getX();
}

double Front::readCurrentTime(){
    auto st = ieskfPtr->getX();
    return st.timestamp.sec();
}

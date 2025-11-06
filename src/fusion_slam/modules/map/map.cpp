/*
 * @Author: ylh 
 * @Date: 2024-04-20 11:45:11 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 23:42:52
 */

#include "fusion_slam/modules/map/map.h"


Map::Map(const std::string &configPath){
    localMapPtr = pcl::make_shared<PCLPointCloud>();
    kdtreePtr = pcl::make_shared<KDTree>();
    ikdtreePtr = std::make_shared<IKDTree>();
    globalMapPtr = pcl::make_shared<PCLPointCloud>();
    globalCloudPtr = pcl::make_shared<PCLPointCloud>();
    globalMapFilterResultPtr = pcl::make_shared<PCLPointCloud>();
    halfLocalMapLength = 50.0;
    saveMap = false;
    savePath = "";
    mapResolution = 0.5;
    localmapInitialized = false;
    cubeLen = 1000.0;
    MOV_THRESHOLD = 1.5f;
    DET_RANGE = 100.0f;
    filterSizeMapMin = 0.5;
    YAML::Node configNode;
    configNode = YAML::LoadFile(configPath);
    mapResolution = configNode["map"]["map_resolution"].as<double>();
    halfLocalMapLength = configNode["map"]["local_map_length"].as<double>();
    halfLocalMapLength *= 0.5;
    saveMap = configNode["map"]["save_map"].as<bool>();
    savePath = configNode["map"]["save_path"].as<std::string>();
    float mapLeafSize = 0.2;
    mapLeafSize = configNode["map"]["map_leaf_size"].as<float>();
    globalMapPtrFilter.setLeafSize(mapLeafSize, mapLeafSize, mapLeafSize);
    LOG(INFO) <<"\033[1;32m"<< "map: " << "\033[0m" << std::endl;
    std::cout << "      map_resolution  : " << mapResolution << std::endl;
    std::cout << "      local_map_length: " << halfLocalMapLength*2.0 << std::endl;
    std::cout << "      save_map        : " << saveMap << std::endl;
    std::cout << "      save_path       : " << savePath << std::endl;
    std::cout << "      map_leaf_size   : " << mapLeafSize << std::endl;
}

Map::~Map(){}

int Map::addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose){
    
        PCLPointCloud globalCloud;
        pcl::transformPointCloud(*cloud, globalCloud, quaternionTrans2Mat4d(rot, pose).cast<float>());
        if(saveMap){
            *globalCloudPtr = globalCloud;
            *globalMapPtr += *globalCloudPtr;
        }
        if(localMapPtr->points.empty()) {*localMapPtr = globalCloud;}
        else{
            for (auto &&point : globalCloud) {
                std::vector<int> ind;
                std::vector<float> distance;
                kdtreePtr->nearestKSearch(point, 5, ind, distance);
                if (distance[0] > mapResolution) localMapPtr->emplace_back(point);
            }         
            auto isInBox = [&](const PointType& pt) {
            return std::abs(pt.x - pose[0]) < halfLocalMapLength &&
            std::abs(pt.y - pose[1]) < halfLocalMapLength &&
            std::abs(pt.z - pose[2]) < halfLocalMapLength;
            };
            auto it = std::partition(localMapPtr->points.begin(), localMapPtr->points.end(), isInBox);
            localMapPtr->points.resize(it-localMapPtr->points.begin());
            localMapPtr->width = localMapPtr->points.size();
            localMapPtr->height = 1;
            localMapPtr->is_dense = true;
        }
        kdtreePtr->setInputCloud(localMapPtr);
        return 0;
}


void Map::lasermapFovSegment(const Eigen::Vector3d& pose)
{
    cubNeedrm.clear();
    Eigen::Vector3d posLiD = pose;
    if (!localmapInitialized){
        for (int i = 0; i < 3; i++){
            localMapPoints.vertex_min[i] = posLiD(i) - cubeLen / 2.0;
            localMapPoints.vertex_max[i] = posLiD(i) + cubeLen / 2.0;
        }
        localmapInitialized = true;
        return;
    }
    float distToMapEdge[3][2];
    bool needMove = false;
    for (int i = 0; i < 3; i++){
        distToMapEdge[i][0] = fabs(posLiD(i) - localMapPoints.vertex_min[i]);
        distToMapEdge[i][1] = fabs(posLiD(i) - localMapPoints.vertex_max[i]);
        if (distToMapEdge[i][0] <= MOV_THRESHOLD * DET_RANGE || distToMapEdge[i][1] <= MOV_THRESHOLD * DET_RANGE) needMove = true;
    }
    if (!needMove) return;
    BoxPointType newLocalMapPoints, tmpBoxpoints;
    newLocalMapPoints = localMapPoints;
    float movDist = max((cubeLen - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmpBoxpoints = localMapPoints;
        if (distToMapEdge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            newLocalMapPoints.vertex_max[i] -= movDist;
            newLocalMapPoints.vertex_min[i] -= movDist;
            tmpBoxpoints.vertex_min[i] = localMapPoints.vertex_max[i] - movDist;
            cubNeedrm.push_back(tmpBoxpoints);
        } else if (distToMapEdge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            newLocalMapPoints.vertex_max[i] += movDist;
            newLocalMapPoints.vertex_min[i] += movDist;
            tmpBoxpoints.vertex_max[i] = localMapPoints.vertex_min[i] + movDist;
            cubNeedrm.push_back(tmpBoxpoints);
        }
    }
    localMapPoints = newLocalMapPoints;
    if(cubNeedrm.size() > 0) ikdtreePtr->Delete_Point_Boxes(cubNeedrm);
}

float Map::calcDist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void Map::mapIncremental(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose, const std::vector<IKDTree::PointVector>& nearstPoints)
{
    auto featsDownSize = cloud->points.size();
    const int NUM_MATCH_POINTS = 5;
    PCLPointCloudPtr featsDownWorld = pcl::make_shared<PCLPointCloud>();
    pcl::transformPointCloud(*cloud, *featsDownWorld, quaternionTrans2Mat4d(rot, pose).cast<float>());
    IKDTree::PointVector PointToAdd;
    IKDTree::PointVector PointNoNeedDownsample;
    PointToAdd.reserve(featsDownSize);
    PointNoNeedDownsample.reserve(featsDownSize);
    for (int i = 0; i < featsDownSize; i++)
    {
        if (!nearstPoints[i].empty())
        {
            const IKDTree::PointVector &pointsNear = nearstPoints[i];
            bool needAdd = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(featsDownWorld->points[i].x/filterSizeMapMin)*filterSizeMapMin + 0.5 * filterSizeMapMin;
            mid_point.y = floor(featsDownWorld->points[i].y/filterSizeMapMin)*filterSizeMapMin + 0.5 * filterSizeMapMin;
            mid_point.z = floor(featsDownWorld->points[i].z/filterSizeMapMin)*filterSizeMapMin + 0.5 * filterSizeMapMin;
            float dist  = calcDist(featsDownWorld->points[i],mid_point);
            if (fabs(pointsNear[0].x - mid_point.x) > 0.5 * filterSizeMapMin && fabs(pointsNear[0].y - mid_point.y) > 0.5 * filterSizeMapMin && fabs(pointsNear[0].z - mid_point.z) > 0.5 * filterSizeMapMin){
                PointNoNeedDownsample.push_back(featsDownWorld->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (pointsNear.size() < NUM_MATCH_POINTS) break;
                if (calcDist(pointsNear[readd_i], mid_point) < dist)
                {
                    needAdd = false;
                    break;
                }
            }
            if (needAdd) PointToAdd.push_back(featsDownWorld->points[i]);
        }
        else
        {
            PointToAdd.push_back(featsDownWorld->points[i]);
        }
    }

    ikdtreePtr->Add_Points(PointToAdd, true);
    ikdtreePtr->Add_Points(PointNoNeedDownsample, false); 
}



int Map::addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose, const std::vector<IKDTree::PointVector>& nearstPoints){
    
        PCLPointCloud globalCloud;
        pcl::transformPointCloud(*cloud, globalCloud, quaternionTrans2Mat4d(rot, pose).cast<float>());
        if(saveMap){
            *globalCloudPtr = globalCloud;
            *globalMapPtr += *globalCloudPtr;
        }
        if(localMapPtr->points.empty()) {
            *localMapPtr = globalCloud;
            ikdtreePtr->set_downsample_param(filterSizeMapMin);
            ikdtreePtr->Build(localMapPtr->points);
            lasermapFovSegment(pose);
            return 0;
        }
        else{
            mapIncremental(cloud, rot, pose, nearstPoints);
            lasermapFovSegment(pose);
            return 0;
        }
        return 0;
}

int Map::reset(){
    localMapPtr->clear();
    globalMapPtr->clear();
    return 0;
}

PCLPointCloudConstPtr Map::getLocalMap(){
    return localMapPtr;
}

KDTreeConstPtr Map::readKDtree(){
    return kdtreePtr;
}

IKDTreePtr Map::readIkdtree(){
    return ikdtreePtr;
}

void Map::saveGlobalMap(){
    if(!saveMap) return;
    if(globalMapPtr->points.empty()){
        LOG(INFO) << "global map is empty" << std::endl;
        return;
    }
    globalMapPtrFilter.setInputCloud(globalMapPtr);
    globalMapPtrFilter.filter(*globalMapFilterResultPtr);
    if (pcl::io::savePCDFileASCII(savePath, *globalMapFilterResultPtr) == -1)
    {
        LOG(INFO) << " save fail ! " << std::endl;
    }else{
        LOG(INFO) << " save pcd success, save_path " << savePath << std::endl;
    }
}

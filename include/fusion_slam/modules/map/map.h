/*
 * @Author: ylh 
 * @Date: 2024-04-20 10:37:19 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:38:01
 */
#ifndef MAP_H
#define MAP_H
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/math/math.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

class Map{
private:
    PCLPointCloudPtr localMapPtr;
    KDTreePtr kdtreePtr;
    IKDTreePtr ikdtreePtr;
    double mapResolution;
    double halfLocalMapLength;
    bool saveMap;
    std::string savePath;
    PCLPointCloudPtr globalMapPtr;
    PCLPointCloudPtr globalCloudPtr;
    PCLPointCloudPtr globalMapFilterResultPtr;
    VoxelFilter globalMapPtrFilter;

    // ikdtree fastlio
    vector<BoxPointType> cubNeedrm;
    BoxPointType localMapPoints;
    bool localmapInitialized;
    double cubeLen;
    float MOV_THRESHOLD;
    float DET_RANGE;
    double filterSizeMapMin;
    float calcDist(PointType p1, PointType p2);
    void lasermapFovSegment(const Eigen::Vector3d& pose);
    void mapIncremental(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose, const std::vector<IKDTree::PointVector>& nearstPoints);

public:
    using Ptr = std::shared_ptr<Map>;
    Map(const std::string &configPath);
    ~Map();
    int addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose);
    int addPcdAndUpdateLocalMap(PCLPointCloudPtr& cloud, const Eigen::Quaterniond& rot, const Eigen::Vector3d& pose, const std::vector<IKDTree::PointVector>& nearstPoints);
    int reset();
    PCLPointCloudConstPtr getLocalMap();
    KDTreeConstPtr readKDtree();
    IKDTreePtr readIkdtree();
    void saveGlobalMap();
};
#endif

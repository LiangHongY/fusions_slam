/*
 * @Author: ylh 
 * @Date: 2024-04-07 22:27:09 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2024-06-04 22:52:57
 */

#ifndef FRONT_H
#define FRONT_H
#include "fusion_slam/type/imu_type.h"
#include "fusion_slam/type/pose.h"
#include "fusion_slam/type/pointcloud.h"
#include "fusion_slam/type/measure_group.h"
#include "fusion_slam/type/base_type.h"
#include "fusion_slam/modules/ieskf/ieskf.h"
#include "fusion_slam/modules/sensors/imu/propagate.h"
#include "fusion_slam/modules/map/map.h"
#include "fusion_slam/modules/sensors/imu/imu_static_init.h"
#include <yaml-cpp/yaml.h>
#include <deque>
class Front {
private:
    std::deque<ImuType> imus;
    std::deque<PointCloud> clouds;
    std::deque<Pose> rtkPoses;
    bool inited;
    IESKF::Ptr ieskfPtr;
    Propagate::Ptr propagatePtr;
    ImuStaticInit::Ptr imuStaticInitPtr;
    double imuAccScale;
    VoxelFilter pcdVoxelFilter;
    PCLPointCloudPtr pcdPtrFilterResult;
    bool initUseRtk;
    double imuInitDuration;
    std::vector<IKDTree::PointVector> nearstPoints;

public:
    using Ptr = std::shared_ptr<Front>;
    Map::Ptr mapPtr;
    Front(const std::string &configPath);
    ~Front();
    int addImu(const ImuType& imu);
    int addLidar(const PointCloud& cloud);
    int addRtk(const Pose& pose);
    bool run();
    int init(MeasureGroupAdd &msg);
    bool syncMeasureGroupAdd(MeasureGroupAdd &msg);

    const PCLPointCloud& readCurrentPointCloud();
    const PCLPointCloud& readCurrentLocalMap();
    StateX readState();
    double readCurrentTime();
};
#endif

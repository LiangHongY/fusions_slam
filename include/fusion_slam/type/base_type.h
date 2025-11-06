/*
 * @Author: ylh 
 * @Date: 2024-04-21 21:25:58 
 * @Last Modified by:   ylh 2252512364@qq.com 
 * @Last Modified time: 2024-04-21 21:25:58 
 */

#ifndef BASE_TYPE_H
#define BASE_TYPE_H
#include "fusion_slam/type/pointcloud.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include "fusion_slam/type/point_type.h"
#include "ikd-Tree/ikd_Tree.hpp"
#include "ivox3d/ivox3d.h"

using IKDTree = KD_TREE<PointType>;
using IKDTreePtr = IKDTree::Ptr;
using IKDTreeConstPtr = std::shared_ptr<const IKDTree>;

// using IVoxelGrid = IVox3D<PointType>;
// using IVoxelGridPtr = IVoxelGrid::Ptr;

using VoxelFilter = pcl::VoxelGrid<PointType>;
using KDTree = pcl::KdTreeFLANN<PointType>;
using KDTreePtr = KDTree::Ptr;
using KDTreeConstPtr = KDTree::ConstPtr;

const double GRAVITY = 9.81;
template<typename _first, typename _second, typename _thrid>
struct triple{
    _first first;
    _second second;
    _thrid thrid;
};
#endif


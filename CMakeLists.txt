cmake_minimum_required(VERSION 3.10)
project(fusion_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" ) # violence speed up! best
# set(CMAKE_CXX_FLAGS "-std=c++17 -Ofast") # better
# set(CMAKE_CXX_FLAGS "-std=c++17 -Ofast -march=native -funroll-loops") # better
# add_compile_options(-std=c++17)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=6)

find_package(OpenMP QUIET)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  novatel_msgs
)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
pkg_check_modules(GLOG REQUIRED libglog)

catkin_package()
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)
set(IESKF_FILE
    src/fusion_slam/modules/ieskf/ieskf.cpp
    src/fusion_slam/modules/front/front.cpp
    src/fusion_slam/modules/sensors/imu/propagate.cpp
    src/fusion_slam/modules/sensors/imu/imu_static_init.cpp
    src/fusion_slam/modules/map/map.cpp
    src/fusion_slam/modules/sensors/rtk/rtk_switch.cpp
    src/fusion_slam/math/math.cpp
)
set(WRAPPER_FILE
    src/app/fusion_slam_node.cpp
    src/wrapper/ros/fusion_slam_ros_wrapper.cpp
    src/wrapper/ros/imu_process/imu_process.cpp
)
set(LOC_FILE
    src/app/location_node.cpp
    src/wrapper/ros/location_ros_wrapper.cpp
    src/wrapper/ros/imu_process/imu_process.cpp
    src/fusion_slam/modules/ieskf/ieskf.cpp
    src/fusion_slam/modules/location/location.cpp
    src/fusion_slam/modules/sensors/imu/propagate.cpp
    src/fusion_slam/modules/sensors/imu/imu_static_init.cpp
    src/fusion_slam/modules/map/map_engine.cpp
    src/fusion_slam/modules/sensors/rtk/rtk_switch.cpp
    src/fusion_slam/math/math.cpp
)
add_executable(${PROJECT_NAME}_node
              ${WRAPPER_FILE}
              ${IESKF_FILE}
              )
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  glog
)

add_executable(location_node
              ${LOC_FILE}
              )
target_link_libraries(location_node
  PRIVATE OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  glog
)

set_target_properties(${PROJECT_NAME}_node location_node PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
                     )

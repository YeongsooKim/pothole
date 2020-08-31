#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


// #include <nav_msgs/Odometry.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/common/common_headers.h>

// #include <pcl/registration/ndt.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/search/impl/search.hpp>

// #include <pcl/console/parse.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
// #include <tf/LinearMath/Matrix3x3.h>

// #include "visualization_msgs/MarkerArray.h"


using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using pPointCloudXYZ = PointCloudXYZ::Ptr;
using cPointCloudXYZ = const PointCloudXYZ;
using cpPointCloudXYZ = const pPointCloudXYZ;

using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using pPointCloudXYZRGB = PointCloudXYZRGB::Ptr;
using cPointCloudXYZRGB = const PointCloudXYZRGB;
using cpPointCloudXYZRGB = const pPointCloudXYZRGB;

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using pPointCloudNormal = PointCloudNormal::Ptr;
using cPointCloudNormal = const PointCloudNormal;
using cpPointCloudNormal = const pPointCloudNormal;

using PCLVisualizer = pcl::visualization::PCLVisualizer;
using pPCLVisualizer = PCLVisualizer::Ptr;

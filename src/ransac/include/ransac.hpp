#ifndef __RANSAC__
#define __RANSAC__

#include <iostream>
#include <vector>
#include <omp.h>
#include <algorithm>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// #include "brain_msgs/VehicleState.h"
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using pPointCloudXYZ = PointCloudXYZ::Ptr;
using cPointCloudXYZ = const PointCloudXYZ;
using cpPointCloudXYZ = const pPointCloudXYZ;

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using pPointCloudXYZI = PointCloudXYZI::Ptr;
using cPointCloudXYZI = const PointCloudXYZI;
using cpPointCloudXYZI = const pPointCloudXYZI;

using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using pPointCloudXYZRGB = PointCloudXYZRGB::Ptr;
using cPointCloudXYZRGB = const PointCloudXYZRGB;
using cpPointCloudXYZRGB = const pPointCloudXYZRGB;

using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using pPointCloudNormal = PointCloudNormal::Ptr;
using cPointCloudNormal = const PointCloudNormal;
using cpPointCloudNormal = const pPointCloudNormal;

using ClusterIndices = std::vector<std::vector<int>>;

const unsigned int FILTER_X = 0;
const unsigned int FILTER_Y = 1;
const unsigned int FILTER_Z = 2;

class RANSAC
{
private:
    // Node Handler
	ros::NodeHandle m_node_handler;

	// subscriber
	ros::Subscriber m_sub_pointcloud;

	// publisher
	ros::Publisher m_pub_ground;
	ros::Publisher m_pub_no_ground;

    // Param
    bool m_b_on_discription;
	double m_d_distance_m;
	int m_i_iteration;
    double m_d_xmax_m;
    double m_d_xmin_m;
    double m_d_ymax_m;
    double m_d_ymin_m;
    double m_d_zmax_m;
    double m_d_zmin_m;

    // PointCloud
	PointCloudXYZI m_input_cloud;
	PointCloudXYZI m_passthrough_cloud;
	PointCloudXYZI m_ground_cloud;
	PointCloudXYZI m_no_ground_cloud;
public:
	RANSAC();
	virtual ~RANSAC();

    void Run();

    inline void SetInputCloud(pPointCloudXYZI p_point_cloud) { m_input_cloud = *p_point_cloud;}
    inline void SetPassTroughCloud(pPointCloudXYZI p_point_cloud) { m_passthrough_cloud = *p_point_cloud;}
    inline void SetGroundCloud(pPointCloudXYZI p_point_cloud) { m_ground_cloud = *p_point_cloud;}
    inline void SetNoGroundCloud(pPointCloudXYZI p_point_cloud) { m_no_ground_cloud = *p_point_cloud;}
	inline pPointCloudXYZI GetInputCloud(void) { pPointCloudXYZI p_point_cloud(new PointCloudXYZI(m_input_cloud)); return p_point_cloud;}
	inline pPointCloudXYZI GetPassTroughCloud(void) { pPointCloudXYZI p_point_cloud(new PointCloudXYZI(m_passthrough_cloud)); return p_point_cloud;}
	inline pPointCloudXYZI GetGroundCloud(void) { pPointCloudXYZI p_point_cloud(new PointCloudXYZI(m_ground_cloud)); return p_point_cloud;}
	inline pPointCloudXYZI GetNoGroundCloud(void) { pPointCloudXYZI p_point_cloud(new PointCloudXYZI(m_no_ground_cloud)); return p_point_cloud;}

private:
    // Common function 
    void SetParam(void);
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& p_input_cloud);
    bool Publish(void);

    // RANSAC function
    bool ConvertMsg2Cloud(const sensor_msgs::PointCloud2::ConstPtr& p_input_cloud);        // input pointcloud2 message
    bool PassThrough(cpPointCloudXYZI& p_input_cloud, unsigned int type);
    bool FilterGround(cpPointCloudXYZI& p_input_cloud, double d_distance, int i_iter_times = 200);
};

#endif // __RANSAC__
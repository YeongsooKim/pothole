#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include "data_type.hpp"

struct pose
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

class NDT
{
public:
	NDT();
	virtual ~NDT();

    void Run();

    inline void SetAccumulateCloud(pPointCloudXYZ pPointCloud) { m_accumulateCloud.swap(*pPointCloud);}
    inline void SetInputTargetCloud(pPointCloudXYZ pPointCloud) { m_inputTargetCloud.swap(*pPointCloud);}
    inline void SetInputSourceCloud(pPointCloudXYZ pPointCloud) { m_inputSourceCloud.swap(*pPointCloud);}
	inline pPointCloudXYZ GetAccumulateCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_accumulateCloud)); return pPointCloud;}
	inline pPointCloudXYZ GetInputTargetCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_inputTargetCloud)); return pPointCloud;}
	inline pPointCloudXYZ GetInputSourceCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_inputSourceCloud)); return pPointCloud;}
private:
    // Node Handler
	ros::NodeHandle m_nodeHandler;

	// subscriber
	ros::Subscriber m_subVelodyne;

	// publisher
	ros::Publisher m_pubAccumulateCloud;

    // Param
    bool m_bOnDiscription;
    std::string m_strFrameName;
    double m_dLeafSize_m;
    double m_dXMax;
    double m_dXMin;
    double m_dYMax;
    double m_dYMin;
    double m_dZMax;
    double m_dZMin;

    // PointCloud
	PointCloudXYZ m_accumulateCloud;
	PointCloudXYZ m_inputTargetCloud;
	PointCloudXYZ m_inputSourceCloud;

    bool m_bIsInitSource;

    // NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> m_ndt; 

    Eigen::Matrix4f m_mat4fBase2Local;
    Eigen::Matrix4f m_mat4fLocal2Base;

    pose m_previousPose;
    pose m_currentPose;
    pose m_ndtPose;
    pose m_addedPose;
    pose m_localizerPose;
    
    double m_diff_x = 0.0; 
    double m_diff_y = 0.0; 
    double m_diff_z = 0.0; 
    double m_diff_yaw;  // current_pose - previous_pose

private:
    // Common function 
    void SetParam(void);
    void VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput);
    void Publish(void);

    // NDT function
    bool PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
	bool Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
};

#endif // __PREPROCESS_HPP__
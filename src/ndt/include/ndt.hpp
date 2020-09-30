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
private:
    // Node Handler
	ros::NodeHandle m_nodeHandler;

	// subscriber
	ros::Subscriber m_subVelodyne;
	ros::Subscriber m_subBrain;

	// publisher
	ros::Publisher m_pubAccumulateCloud;

    // Param
    bool m_bOnDiscription;
    bool m_bUseNDT;
    std::string m_strFrameName;
    double m_dLeafSize_m;
    double m_dXMax;
    double m_dXMin;
    double m_dYMax;
    double m_dYMin;
    double m_dZMax;
    double m_dZMin;

    // PointCloud
	PointCloudXYZ m_inputCloud;
	PointCloudXYZ m_accumulateCloud;
	PointCloudXYZ m_targetCloud;
	PointCloudXYZ m_sourceCloud;

    bool m_bIsInitSource;

    // Vehilce state
    double m_dVehicleSpeed_vs;
    double m_dYaw_rad;
    double m_x;
    double m_y;
    double m_z;
    double m_dTimeDiff_s;
    ros::Time m_currTime_s;
    ros::Time m_prevTime_s;

    // NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> m_ndt; 

    Eigen::Matrix4f m_mat4fBase2Local;
    Eigen::Matrix4f m_mat4fLocal2Base;   

    pose m_previousPose;
    pose m_currentPose;
    pose m_ndtPose;
    pose m_addedPose;
    pose m_localizerPose;

    // brain call back
    bool m_bIsBrainCall;
    bool m_bIsFirstBrainCallback;
    
    // For debugging param
    unsigned int m_iComputingIndex;

public:
	NDT();
	virtual ~NDT();

    void Run();

    inline void SetInputCloud(pPointCloudXYZ pPointCloud) { m_inputCloud = *pPointCloud;}
    inline void SetTargetCloud(pPointCloudXYZ pPointCloud) { m_targetCloud = *pPointCloud;}
    inline void SetSourceCloud(pPointCloudXYZ pPointCloud) { m_sourceCloud = *pPointCloud;}
    inline void SetAccumulateCloud(pPointCloudXYZ pPointCloud) { m_accumulateCloud = *pPointCloud;}
	inline pPointCloudXYZ GetInputCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_inputCloud)); return pPointCloud;}
	inline pPointCloudXYZ GetTargetCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_targetCloud)); return pPointCloud;}
	inline pPointCloudXYZ GetSourceCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_sourceCloud)); return pPointCloud;}
	inline pPointCloudXYZ GetAccumulateCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_accumulateCloud)); return pPointCloud;}

private:
    // Common function 
    void SetParam(void);
    void VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput);
    void BrainCallback(const brain_msgs::VehicleState::ConstPtr& pInput);
    void Publish(void);

    // NDT function
    bool ConvertMsg2Cloud(const sensor_msgs::PointCloud2::ConstPtr& pInput);        // input pointcloud2 message
    bool Initialize(void);                                                          // input passthrough cloud
	bool Voxelization(void);                                                        // input passthrough cloud
    bool SetNDTParam(void);                                                         // input downsample cloud
    bool ComputeInitGuess(void);
    bool Update(cpPointCloudXYZ& pInputCloud);                                      // input passthrough cloud
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
};

#endif // __PREPROCESS_HPP__
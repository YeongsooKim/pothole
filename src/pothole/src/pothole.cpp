#include "pothole.hpp"

Pothole::Pothole (void)
{
	// subscriber init
	m_subVelodyne = m_nodeHandler.subscribe ("input", 100000, &Pothole::VelodyneCallback, this);

	// publisher init
    m_pubPreprocessCloud = m_nodeHandler.advertise<sensor_msgs::PointCloud2>("/pothole/preprocess_cloud", 1000);
    m_pubRoadCloud = m_nodeHandler.advertise<sensor_msgs::PointCloud2>("/pothole/road_cloud", 1000);
    m_pubAccumulateCloud = m_nodeHandler.advertise<sensor_msgs::PointCloud2>("/pothole/accumulate_cloud", 1000);
    m_pubColorCloud = m_nodeHandler.advertise<sensor_msgs::PointCloud2>("/pothole/color_cloud", 1000);

    SetParam();

	Preprocess preprocess(m_bOnDiscription, m_strFixedFrameName, m_dLeafSize_m, m_dXMax, m_dXMin, m_dYMax, m_dYMin, m_dZMax, m_dZMin);
	m_preprocess = preprocess;
}

Pothole::~Pothole (void)
{
}

void Pothole::SetParam(void)
{
	m_nodeHandler.getParam("pothole/on_decription", m_bOnDiscription);
	m_nodeHandler.getParam("pothole/fixed_frame_name", m_strFixedFrameName);
	m_nodeHandler.getParam("pothole/voxel_leafsize", m_dLeafSize_m);
	m_nodeHandler.getParam("pothole/x_max", m_dXMax);
	m_nodeHandler.getParam("pothole/x_min", m_dXMin);
	m_nodeHandler.getParam("pothole/y_max", m_dYMax);
	m_nodeHandler.getParam("pothole/y_min", m_dYMin);
	m_nodeHandler.getParam("pothole/z_max", m_dZMax);
	m_nodeHandler.getParam("pothole/z_min", m_dZMin);
	// m_nodeHandler.param ("pothole/threshold_range", m_dMaxScanRange_m, 15.0);
	// m_nodeHandler.param ("pothole/min_travle_distacne", m_dMinAddScanShift, 1.0);
	// m_nodeHandler.param ("pothole/add_threshold", m_dExtraMaxScanRange_m, 3.0);
	// m_nodeHandler.param ("pothole/ndt_epsilon", m_dEpsilon, 0.01);
	// m_nodeHandler.param ("pothole/ndt_stepSize", m_dStepSize, 0.1);
	// m_nodeHandler.param ("pothole/ndt_resolution", m_dResolution_m, 1.0);
	// m_nodeHandler.param ("pothole/last_code", m_bIsLastCode, true);
	// m_nodeHandler.param ("pothole/threshold_high", m_dHighThreshold_m, 0.9);
	// m_nodeHandler.param ("pothole/threshold_low", m_dLowThreshold_m, -1.1);
}

void Pothole::VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput)
{
    // Container for input data 
	PointCloudXYZ tmpCloud;
    tmpCloud.header.frame_id = m_strFixedFrameName;
	pcl::fromROSMsg(*pInput, tmpCloud);
	pPointCloudXYZ pTempCloud (new PointCloudXYZ(tmpCloud));

	ROS_INFO_STREAM("###### 1. Preprocessing ######");
	if (!m_preprocess.run(pTempCloud)) {ROS_ERROR_STREAM("Fail Preprocessing");}
	// ROS_INFO_STREAM("###### 2. Preprocessing ######");
	// ROS_INFO_STREAM("###### 3. Preprocessing ######");
	// ROS_INFO_STREAM("###### 4. Preprocessing ######");


    Publish();
}

void Pothole::Publish(void)
{
    m_pubPreprocessCloud.publish(m_preprocess.GetPreprocessCloud());
    // m_pubRoadCloud.publish();
    // m_pubAccumulateCloud.publish();
    // m_pubColorCloud.publish();
}
#include "preprocess.hpp"

Preprocess::Preprocess() 
{
	// subscriber init
	m_subVelodyne = m_nodeHandler.subscribe ("input", 100000, &Preprocess::VelodyneCallback, this);

	// publisher init
    m_pubPreprocessCloud = m_nodeHandler.advertise<sensor_msgs::PointCloud2>("/pothole/preprocess_cloud", 1000);

    SetParam();
	m_preprocessCloud.header.frame_id = m_strFrameName;
}
Preprocess::~Preprocess()
{

}
void Preprocess::Run()
{
	ros::Rate delay(10);
	
	while(ros::ok()) {
		ros::spinOnce();
		delay.sleep();
	}
}

void Preprocess::SetParam(void)
{
	m_nodeHandler.getParam("preprocess/on_decription", m_bOnDiscription);
	m_nodeHandler.getParam("preprocess/fixed_frame_name", m_strFrameName);
	m_nodeHandler.getParam("preprocess/voxel_leafsize", m_dLeafSize_m);
	m_nodeHandler.getParam("preprocess/x_max", m_dXMax);
	m_nodeHandler.getParam("preprocess/x_min", m_dXMin);
	m_nodeHandler.getParam("preprocess/y_max", m_dYMax);
	m_nodeHandler.getParam("preprocess/y_min", m_dYMin);
	m_nodeHandler.getParam("preprocess/z_max", m_dZMax);
	m_nodeHandler.getParam("preprocess/z_min", m_dZMin);
}
void Preprocess::VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput)
{
    // Container for input data 
	PointCloudXYZ tmpCloud;
	pcl::fromROSMsg(*pInput, tmpCloud);
	pPointCloudXYZ pTempCloud (new PointCloudXYZ(tmpCloud));

    // Thresholding
	pPointCloudXYZ pPassthroughCloud (new PointCloudXYZ);
	if (!PassThrough(pTempCloud, pPassthroughCloud)) ROS_ERROR_STREAM("Fail Pass Through");

	// Voxelization
	pPointCloudXYZ pDownsampledCloud (new PointCloudXYZ);
	if (!Voxelization (pPassthroughCloud, pDownsampledCloud))ROS_ERROR_STREAM("Fail Voxel Grid Filter");
	if (m_bOnDiscription){
		std::cout << "Original: " << pPassthroughCloud->points.size() << " points." << std::endl;
		std::cout << "Filtered: " << pDownsampledCloud->points.size() << " points." << std::endl;
	}

	// Publish
	Publish();

	SetPreprocessCloud(pDownsampledCloud);
}

void Preprocess::Publish(void)
{
    m_pubPreprocessCloud.publish(GetPreprocessCloud());
}

bool Preprocess::PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud)
{
    // Thresholding 
	pcl::PassThrough<pcl::PointXYZ> passFilter;
	passFilter.setInputCloud (pInputCloud);
	// passFilter.setFilterFieldName("x");
	// passFilter.setFilterLimits (m_dXMin, m_dXMax); 	
	// passFilter.setFilterFieldName("z");
	// passFilter.setFilterLimits (m_dZMin, m_dZMax);
	passFilter.setFilterFieldName("y");
	passFilter.setFilterLimits (m_dXMin, m_dXMax); 	
	passFilter.filter (*pOutputCloud);	
	
	if (pOutputCloud->empty()) return false;
	else return true;
}

bool Preprocess::Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud)
{
    // Voxel length of the corner : fLeafSize
	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
	voxelFilter.setInputCloud (pInputCloud);
	voxelFilter.setLeafSize(m_dLeafSize_m, m_dLeafSize_m, m_dLeafSize_m);
	voxelFilter.filter (*pOutputCloud);

	if (pOutputCloud->empty()) return false;
	else return true;
}
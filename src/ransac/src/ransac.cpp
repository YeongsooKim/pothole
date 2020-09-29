#include "ransac.hpp"

RANSAC::RANSAC() 
{
	// subscriber init
	m_sub_pointcloud = m_node_handler.subscribe ("/rslidar_points", 100000, &RANSAC::PointCloudCallback, this);

	// publisher init
    m_pub_ground = m_node_handler.advertise<sensor_msgs::PointCloud2>("/pothole/p_ground_cloud", 1000);
    m_pub_no_ground = m_node_handler.advertise<sensor_msgs::PointCloud2>("/pothole/no_ground", 1000);

    SetParam();
}

RANSAC::~RANSAC(){ }

void RANSAC::Run()
{
	ros::Rate delay(10);
	
	while(ros::ok()) {
		ros::spinOnce();
		delay.sleep();
	}
}

void RANSAC::SetParam(void)
{
	m_node_handler.getParam("ransac/on_decription", m_b_on_discription);
	m_node_handler.getParam("ransac/distance", m_d_distance_m);
	m_node_handler.getParam("ransac/iteration", m_i_iteration);
	m_node_handler.getParam("ransac/x_max_m", m_d_xmax_m);
	m_node_handler.getParam("ransac/x_min_m", m_d_xmin_m);
	m_node_handler.getParam("ransac/y_max_m", m_d_ymax_m);
	m_node_handler.getParam("ransac/y_min_m", m_d_ymin_m);
	m_node_handler.getParam("ransac/z_max_m", m_d_zmax_m);
	m_node_handler.getParam("ransac/z_min_m", m_d_zmin_m);
}

void RANSAC::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& p_input_cloud)
{
    if (!ConvertMsg2Cloud(p_input_cloud)) ROS_ERROR_STREAM("Failed Convert Message to PointCloud");
    else if (!PassThrough(GetInputCloud(), FILTER_Z)) ROS_ERROR_STREAM("Failed pass through filter");
    else if (!PassThrough(GetPassTroughCloud(), FILTER_Y)) ROS_ERROR_STREAM("Failed pass through filter");
    else if (!FilterGround(GetPassTroughCloud(), m_d_distance_m, m_i_iteration)) ROS_ERROR_STREAM("Failed ransac");

	// Publish
	Publish();
}

void RANSAC::Publish(void)
{
    m_pub_ground.publish(GetGroundCloud());
    m_pub_no_ground.publish(GetNoGroundCloud());
}

bool RANSAC::ConvertMsg2Cloud(const sensor_msgs::PointCloud2::ConstPtr& p_input_cloud)
{
	PointCloudXYZI tmp_cloud;
	pcl::fromROSMsg(*p_input_cloud, tmp_cloud);
	pPointCloudXYZI p_tmp_cloud (new PointCloudXYZI(tmp_cloud));

	if (p_tmp_cloud->empty()) return false;

	SetInputCloud(p_tmp_cloud);
	return true;
}

bool RANSAC::PassThrough(cpPointCloudXYZI& p_input_cloud, unsigned int type)
{
    // Thresholding 
	pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pPointCloudXYZI p_passthrough_cloud (new PointCloudXYZI);
	pass_filter.setInputCloud (p_input_cloud);
    switch (type){
        case FILTER_X:
            pass_filter.setFilterFieldName("x");
	        pass_filter.setFilterLimits (m_d_xmin_m, m_d_xmax_m);
            break;
        case FILTER_Y:
            pass_filter.setFilterFieldName("y");
	        pass_filter.setFilterLimits (m_d_ymin_m, m_d_ymax_m);
            break;
        case FILTER_Z:
            pass_filter.setFilterFieldName("z");
	        pass_filter.setFilterLimits (m_d_zmin_m, m_d_zmax_m);
            break;
        default:
            break;
    }
	pass_filter.filter (*p_passthrough_cloud);	
	
	if (p_passthrough_cloud->empty()) return false;
    
    SetPassTroughCloud(p_passthrough_cloud);
    return true;
}

bool RANSAC::FilterGround(cpPointCloudXYZI& p_input_cloud, double d_distance, int i_iter_times)
{
    pcl::ModelCoefficients::Ptr p_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr p_inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(m_d_distance_m);
    seg.setMaxIterations(m_i_iteration);
    seg.setInputCloud(p_input_cloud);

    seg.segment(*p_inliers, *p_coefficients);

    pPointCloudXYZI p_landscape_cloud(new PointCloudXYZI), p_ground_cloud(new PointCloudXYZI);
    // extract points
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(p_input_cloud);
    extract.setIndices(p_inliers);
    extract.setNegative(true);
    extract.filter(*p_landscape_cloud);
	if (p_landscape_cloud->empty()) return false;

    extract.setNegative(false);
    extract.filter(*p_ground_cloud);
	if (p_ground_cloud->empty()) return false;

    SetGroundCloud(p_ground_cloud);
    SetNoGroundCloud(p_landscape_cloud);
    return true;
}
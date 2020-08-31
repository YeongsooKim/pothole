#include "preprocess.hpp"

Preprocess::Preprocess(std::string strFrameName, double dLeafSize, double dXMax, double dXMin, double dYMax, double dYMin, double dZMax, double dZMin) : 
m_strFrameName(strFrameName), 
m_dLeafSize_m(dLeafSize),
m_dXMax(dXMax),
m_dXMin(dXMin),
m_dYMax(dYMax),
m_dYMin(dYMin),
m_dZMax(dZMax),
m_dZMin(dZMin)
{
    m_preprocessCloud.header.frame_id = m_strFrameName;
}
Preprocess::~Preprocess()
{

}
bool Preprocess::run(cpPointCloudXYZ& pInputCloud)
{
    // Thresholding
	pPointCloudXYZ pPassthroughCloud (new PointCloudXYZ);
	PassThrough(pInputCloud, pPassthroughCloud);
	if (pPassthroughCloud->empty()) return false;

	// Voxelization
	pPointCloudXYZ pDownsampledCloud (new PointCloudXYZ);
	Voxelization (pPassthroughCloud, pDownsampledCloud);
	if (pDownsampledCloud->empty()) return false;

	SetPreprocessCloud(pDownsampledCloud);
	return true;
}

bool Preprocess::PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud)
{
    // Thresholding 
	pcl::PassThrough<pcl::PointXYZ> passFilter;
	passFilter.setInputCloud (pInputCloud);
	// passFilter.setFilterFieldName("x");
	// passFilter.setFilterLimits (m_dXMin, m_dXMax); 	
	passFilter.setFilterFieldName("y");
	passFilter.setFilterLimits (-3.0, 3.0);
	// passFilter.setFilterFieldName("z");
	// passFilter.setFilterLimits (m_dZMin, m_dZMax);
	passFilter.filter (*pOutputCloud);	

	return true;
}

bool Preprocess::Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud)
{
    // Voxel length of the corner : fLeafSize
	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
	voxelFilter.setInputCloud (pInputCloud);
	voxelFilter.setLeafSize(m_dLeafSize_m, m_dLeafSize_m, m_dLeafSize_m);
	voxelFilter.filter (*pOutputCloud);

	return true;
}
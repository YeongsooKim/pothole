#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include "data_type.hpp"

class Preprocess
{
public:
	Preprocess();
	virtual ~Preprocess();

    void Run();

    inline void SetPreprocessCloud(pPointCloudXYZ pPointCloud) { m_preprocessCloud.swap(*pPointCloud);}
	inline pPointCloudXYZ GetPreprocessCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_preprocessCloud)); return pPointCloud;}
private:
    // Node Handler
	ros::NodeHandle m_nodeHandler;

	// subscriber
	ros::Subscriber m_subVelodyne;

	// publisher
	ros::Publisher m_pubPreprocessCloud;

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
	PointCloudXYZ m_preprocessCloud;

private:
    // Common function 
    void SetParam(void);
    void VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput);
    void Publish(void);

    // Preprocess function
    bool PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
	bool Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
};

#endif // __PREPROCESS_HPP__
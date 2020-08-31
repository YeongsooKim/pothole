#ifndef __POTHOLE_HPP__
#define __POTHOLE_HPP__

#include <fstream>
#include <thread>
#include <vector>
#include <map>

#include "data_type.hpp"
#include "preprocess.hpp"

class Pothole
{
private:
    // Node Handler
	ros::NodeHandle m_nodeHandler;

	// subscriber
	ros::Subscriber m_subVelodyne;

	// publisher
	ros::Publisher m_pubPreprocessCloud;
	ros::Publisher m_pubRoadCloud;
	ros::Publisher m_pubAccumulateCloud;
	ros::Publisher m_pubColorCloud;

    // Parameter
	std::string m_strFixedFrameName;
	double m_dLeafSize_m;
	double m_dXMax;
	double m_dXMin;
	double m_dYMax;
	double m_dYMin;
	double m_dZMax;
	double m_dZMin;

	// process class
	Preprocess m_preprocess;

public:
	Pothole();
	virtual ~Pothole();

private:
    void SetParam(void);
    void VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& pInput);
    // void Publish(cpPointCloudXYZ& input);
    void Publish(void);
};

#endif // __POTHOLE_HPP__
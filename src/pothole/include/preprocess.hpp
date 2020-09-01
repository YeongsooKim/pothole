#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include "data_type.hpp"

class Preprocess
{
public:
	Preprocess() = default;
    Preprocess(bool bOnDiscription, std::string strFrameName, double dLeafSize, double dXMax, double dXMin, double dYMax, double dYMin, double dZMax, double dZMin);

	virtual ~Preprocess();

    bool run(cpPointCloudXYZ& pInputCloud);

    inline void SetPreprocessCloud(pPointCloudXYZ pPointCloud) { m_preprocessCloud.swap(*pPointCloud);}
	inline pPointCloudXYZ GetPreprocessCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_preprocessCloud)); return pPointCloud;}
private:
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

    // Pointcloud
	PointCloudXYZ m_preprocessCloud;

    bool PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
	bool Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
};

#endif // __PREPROCESS_HPP__
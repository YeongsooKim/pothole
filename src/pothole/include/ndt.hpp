#ifndef __NDT_HPP__
#define __NDT_HPP__

#include "data_type.hpp"

class NDT
{
public:
	NDT() = default;
    NDT(std::string strFrameName, double dLeafSize, double dXMax, double dXMin, double dYMax, double dYMin, double dZMax, double dZMin);

	virtual ~NDT();

    bool run(cpPointCloudXYZ& pInputCloud);

    inline void SetAccumulateCloud(pPointCloudXYZ pPointCloud) { m_accumulateCloud.swap(*pPointCloud);}
	inline pPointCloudXYZ GetAccumulateCloud(void) { pPointCloudXYZ pPointCloud(new PointCloudXYZ(m_accumulateCloud)); return pPointCloud;}
private:
    // Param
    std::string m_strFrameName;
    double m_dLeafSize_m;
    double m_dXMax;
    double m_dXMin;
    double m_dYMax;
    double m_dYMin;
    double m_dZMax;
    double m_dZMin;

    // Pointcloud
	PointCloudXYZ m_accumulateCloud;

    void Registration(cpPointCloudXYZ& pSourceCloud,pPointCloudXYZ& pTargetCloud);
    bool PassThrough(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
	bool Voxelization(cpPointCloudXYZ& pInputCloud,pPointCloudXYZ& pOutputCloud);
};


#endif // __NDT_HPP__
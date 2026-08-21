#ifndef FEATURE_EXTRACTION_RESIDUAL_H
#define FEATURE_EXTRACTION_RESIDUAL_H

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "math_utils.hpp"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;


class FeatureExtractionResidual {
public:
    FeatureExtractionResidual(PointCloudXYZI &scan, Mat3D &rot, Vec3D &pos);
    ~FeatureExtractionResidual();

    void ScanHandler();
    void CaculateCurvature();
    void FeatureExtraction();

    void computeResidual();


private:
    PointCloudXYZI::Ptr scan_;
    PointCloudXYZI::Ptr corner_;
    PointCloudXYZI::Ptr surface_;

    Mat3D rot_;
    Vec3D pos_;

    


};



#endif

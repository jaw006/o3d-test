#include "PointCloud.h"

Reco3D::PointCloud::PointCloud(std::shared_ptr<Reco3D::o3d_PointCloud> points, 
        std::shared_ptr<Reco3D::RGBDCapture_t> capture)
    :   points_(points),
        capture_(capture)
{

}
Reco3D::PointCloud::PointCloud()
    :   points_(nullptr),
        capture_(std::make_shared<Reco3D::RGBDCapture_t>())
{

}

Reco3D::PointCloud::~PointCloud()
{

}

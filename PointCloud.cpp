#include "PointCloud.h"

Reco3D::PointCloud::PointCloud(std::shared_ptr<Reco3D::o3d_PointCloud> points, 
        std::shared_ptr<Reco3D::RGBDCapture_t> capture)
    :   points_(points),
        capture_(capture)
{

}

//Reco3D::PointCloud::PointCloud(PointCloud& p2) 
//    : points_(std::make_shared<o3d_PointCloud>(p2.GetPoints())),
//      capture_(std::make_shared<RGBDCapture_t>(p2.GetCapture()))
//{
//}

Reco3D::PointCloud::PointCloud()
    :   points_(std::make_shared<Reco3D::o3d_PointCloud>()),
        capture_(std::make_shared<Reco3D::RGBDCapture_t>())
{

}

Reco3D::PointCloud::~PointCloud()
{

}

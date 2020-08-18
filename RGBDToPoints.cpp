#include "RGBDToPoints.h"

Reco3D::RGBDToPoints::RGBDToPoints()
{
    points_ = std::make_shared<PointCloud>();
}

Reco3D::RGBDToPoints::~RGBDToPoints()
{
}

std::shared_ptr<PointCloud> Reco3D::RGBDToPoints::ConvertToPointCloud(std::shared_ptr<RGBDImage> image)
{
    if (image == nullptr)
    {
        return points_;
    }
    auto& color = image->color_;
    auto& depth = image->depth_;
    auto rgbd_image = RGBDImage::CreateFromColorAndDepth(color, depth, 1.0, 1000.0, false);
//    const RGBDImage img = *rgbd_image;
    PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);
    points_ = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_image, intrinsic);
    return points_;
}


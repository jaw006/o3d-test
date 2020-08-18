#include "RGBDToPoints.h"

Reco3D::RGBDToPoints::RGBDToPoints()
{
    points_ = std::make_shared<PointCloud>();
}

Reco3D::RGBDToPoints::~RGBDToPoints()
{
}

std::shared_ptr<PointCloud> Reco3D::RGBDToPoints::ConvertToPointCloud(std::shared_ptr<RGBDImage>& image)
{
    if (image == nullptr)
    {
        return points_;
    }
    auto rgbd_image = MakeNewRGBDImage(image);
//    const RGBDImage img = *rgbd_image;
    PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);
    points_ = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_image, intrinsic);
    return points_;
}

std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::MakeNewRGBDImage(std::shared_ptr<RGBDImage>& image)
{
    auto& color = image->color_;
    auto& depth = image->depth_;
    return RGBDImage::CreateFromColorAndDepth(color, depth, 1.0, 10000.0, false);
}

std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::SaveImage(std::shared_ptr<RGBDImage>& image)
{
    image_ = MakeNewRGBDImage(image);
    return image_;
}





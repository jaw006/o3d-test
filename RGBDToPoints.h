#pragma once

#include <Open3D/Open3D.h>
// Takes a open3d::Geometry::RGBDImage and converts it into points
using namespace open3d::camera;
using namespace open3d::geometry;

namespace Reco3D
{

    class RGBDToPoints
    {
    public:
        RGBDToPoints();
        ~RGBDToPoints();
        std::shared_ptr<PointCloud> ConvertToPointCloud(std::shared_ptr<RGBDImage>& image);
        std::shared_ptr<open3d::geometry::RGBDImage> MakeNewRGBDImage(std::shared_ptr<RGBDImage>& image);
        std::shared_ptr<open3d::geometry::RGBDImage> SaveImage(std::shared_ptr<RGBDImage>& image);
        void SetPose(Eigen::Matrix4d& pose);

        std::shared_ptr<PointCloud> points_;
        std::shared_ptr<RGBDImage> image_;
        Eigen::Matrix4d pose_;

    };
}

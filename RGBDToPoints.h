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
    protected:
        std::shared_ptr<PointCloud> points_;

    };
}

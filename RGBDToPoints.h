#pragma once

#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>

// Takes a open3d::Geometry::RGBDImage and converts it into points
using namespace open3d::camera;
using namespace open3d::geometry;

#define DATA_DIR "data/"

namespace Reco3D
{

    class RGBDToPoints
    {
    public:
        std::shared_ptr<PointCloud> points_;
        std::shared_ptr<RGBDImage> image_;
        Eigen::Matrix4d pose_;
        Eigen::Matrix3d rotation_;
        Eigen::Vector3d position_;

        RGBDToPoints();
        ~RGBDToPoints();
        std::shared_ptr<PointCloud> ConvertToPointCloud(std::shared_ptr<RGBDImage>& image);
        std::shared_ptr<open3d::geometry::RGBDImage> MakeNewRGBDImage(std::shared_ptr<RGBDImage>& image);
        std::shared_ptr<open3d::geometry::RGBDImage> SaveImage(std::shared_ptr<RGBDImage>& image);
        void SetPose(Eigen::Matrix4d& pose);
        bool ExportCapture(std::string filename);
        bool ExportPLY(std::string filename);
        bool ExportPose(std::string filename);
        bool ReadPoseFromFile(std::string filename);
    };
}

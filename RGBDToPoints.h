#pragma once

#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>
#include "RGBD_IO.h"
#include "PointCloud.h"
#include "Types.h"

// Takes a open3d::Geometry::RGBDImage and converts it into points
using namespace open3d::camera;
//using namespace open3d::geometry;

#define DATA_DIR "data/"
//PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);

namespace Reco3D
{
    // Factory class which converts a RGBDCapture_t struct to a point cloud
    // Also has methods to serialize point clouds
    class RGBDToPoints
    {
    public:
//        std::shared_ptr<Reco3D::PointCloud> pointCloud_;
//        std::shared_ptr<PointCloud> points_;
//        std::shared_ptr<RGBDImage> image_;
//        Eigen::Matrix4d pose_;
//        Eigen::Matrix3d rotation_;
//////////////////        Eigen::Vector3d position_;

        RGBDToPoints();
        ~RGBDToPoints();

        std::shared_ptr<PointCloud> ToPointCloud(std::shared_ptr<RGBDCapture_t> capture);

        bool ExportCapture(std::string filename, std::shared_ptr<o3d_PointCloud> points, std::shared_ptr<Reco3D::RGBDCapture_t> capture);
        bool ExportPLY(std::string filename, std::shared_ptr<Reco3D::o3d_PointCloud> points);
        bool ExportPose(std::string filename, std::shared_ptr<Reco3D::RGBDCapture_t> capture);
        Reco3D::ImagePose ReadPoseFromFile(std::string filename);

//        void SetPose(Eigen::Matrix4d& pose);
//        bool ExportPLY(std::string filename);
//        std::shared_ptr<PointCloud> ConvertToPointCloud(std::shared_ptr<RGBDImage>& image);
//        std::shared_ptr<open3d::geometry::RGBDImage> SaveImage(std::shared_ptr<RGBDImage>& image);
    private:
        std::shared_ptr<open3d::geometry::RGBDImage> MakeNewRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage>& image);
    };
}

#pragma once

#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>
#include "RGBD_IO.h"
#include "PointCloud.h"
#include "Types.h"

// Takes a open3d::Geometry::RGBDImage and converts it into points
using namespace open3d::camera;

#define DATA_DIR "data/"
//PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);

namespace Reco3D
{
    // Factory class which converts a RGBDCapture_t struct to a point cloud
    // Also has methods to serialize point clouds
    class RGBDToPoints
    {
    public:
        RGBDToPoints();
        ~RGBDToPoints();

        std::shared_ptr<PointCloud> ToPointCloud(std::shared_ptr<RGBDCapture_t> capture);

        bool ExportCapture(std::string filename, std::shared_ptr<o3d_PointCloud> points, std::shared_ptr<Reco3D::RGBDCapture_t> capture);
        bool ExportPLY(std::string filename, std::shared_ptr<Reco3D::o3d_PointCloud> points);
        bool ExportPose(std::string filename, std::shared_ptr<Reco3D::RGBDCapture_t> capture);
        Reco3D::ImagePose ReadPoseFromFile(std::string filename);
        // TODO
//        std::shared_ptr<open3d::geometry::RGBDImage> SaveImage(std::shared_ptr<RGBDImage>& image);

    private:
        std::shared_ptr<open3d::geometry::RGBDImage> MakeNewRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage>& image);
    };
}

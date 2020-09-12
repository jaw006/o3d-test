#pragma once

#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>
#include "RGBD_IO.h"
#include "PointCloud.h"
#include "Types.h"

// Azure Kinect Intrinsic Parameters
// intrinsic.json
//
// "color_mode" : "MJPG_720P",
// "depth_mode" : "WFOV_2X2BINNED",
// "height" : 720,
// "intrinsic_matrix" :
// 	[
// 		612.31494140625,
// 		0.0,
// 		0.0,
// 		0.0,
// 		612.007080078125,
// 		0.0,
// 		636.7769775390625,
// 		370.02471923828125,
// 		1.0
// 	] ,
// 	"serial_number_" : "000307401412",
// 	"stream_length_usec" : 14933344,
// 	"width" : 1280
// }
#define INTRINSIC_FX 612.31494140625
#define INTRINSIC_FY 612.007080078125
#define INTRINSIC_CX 636.7769775390625
#define INTRINSIC_CY 370.02471923828125

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
//        bool ExportRGBDImage(std::string filename, std::shared_ptr<Reco3D::RGBDCapture_t> capture);
        Reco3D::ImagePose ReadPoseFromFile(std::string filename);

    private:
        std::shared_ptr<open3d::geometry::RGBDImage> MakeNewRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage>& image);
//        bool OpenFile(std::ofstream& stream, std::string& filepath);
//        bool CloseFile(std::ofstream& stream);


    };
}

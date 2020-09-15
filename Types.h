#pragma once
#include <Open3D/Open3D.h>
#include <Eigen/Core>

namespace Reco3D
{
	// CONSTANTS
	// Camera Intrinsics
    #define CAMERA_RES_X 1280
    #define CAMERA_RES_Y 720
    #define INTRINSIC_FX 612.31494140625
    #define INTRINSIC_FY 612.007080078125
    #define INTRINSIC_CX 636.7769775390625
    #define INTRINSIC_CY 370.02471923828125
	// Meshing
	#define POISSON_DEPTH 16

	typedef Eigen::Matrix4d ImagePose;
	typedef Eigen::Vector4d ImageQuaternion;
	typedef open3d::camera::PinholeCameraTrajectory CameraPose;
	typedef open3d::camera::PinholeCameraParameters CameraPoseParameters;

	typedef std::shared_ptr<open3d::geometry::RGBDImage> ImageRGBD;

	// Struct to encapsulate pose transform and image data
	struct RGBDCapture_t
	{
		RGBDCapture_t() {};
		ImagePose pose_;
		ImageRGBD image_;
		ImageQuaternion quat_;
		CameraPoseParameters camPose_;
	};

    typedef open3d::geometry::PointCloud   o3d_PointCloud;
	typedef open3d::geometry::TriangleMesh o3d_TriMesh;
}
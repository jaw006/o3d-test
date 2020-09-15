#pragma once
#include <Open3D/Open3D.h>
#include <Eigen/Core>

namespace Reco3D
{
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
	};

    typedef open3d::geometry::PointCloud   o3d_PointCloud;
	typedef open3d::geometry::TriangleMesh o3d_TriMesh;
}
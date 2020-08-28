#pragma once
#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>

namespace Reco3D
{
	// Container class that holds the points and a pose
	class PointCloud
	{
	typedef open3d::geometry::PointCloud o3d_PointCloud;
	private:
        std::shared_ptr<o3d_PointCloud> points_;
        Eigen::Matrix4d pose_;
	public:
		PointCloud(std::shared_ptr<PointCloud> points_);
		PointCloud();
		~PointCloud();
		std::shared_ptr<o3d_PointCloud> GetPoints() { return points_; }
        Eigen::Matrix4d GetPose() 		    { return pose_; };
        void SetPose(Eigen::Matrix4d pose)  { pose_ = pose; };
		bool SetPoints(std::shared_ptr<o3d_PointCloud> points) { points_ = points; };
	};
}
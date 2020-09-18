#pragma once
#include <Open3D/Open3D.h>
#include <iostream>
#include <fstream>
#include <memory>
#include "Types.h"

namespace Reco3D
{
	// Container class that holds the points and a pose
	class PointCloud
	{
	private:
        std::shared_ptr<o3d_PointCloud> points_;
		std::shared_ptr<RGBDCapture_t> capture_;
	public:
		PointCloud(std::shared_ptr<o3d_PointCloud> points, std::shared_ptr<RGBDCapture_t> capture);
//		PointCloud(PointCloud& p2);
		PointCloud();
		~PointCloud();
        ImagePose GetPose() 		    { return capture_->pose_; };
        ImageQuaternion GetQuat() 		{ return capture_->quat_; };
		std::shared_ptr<RGBDCapture_t> GetCapture() { return capture_; };
//        void SetPose(ImagePose pose)  { capture_->pose_ = pose; };
		void SetPose(ImagePose pose);
        void SetQuat(ImageQuaternion quat )  { capture_->quat_ = quat; };
//		void SetPoints(std::shared_ptr<o3d_PointCloud> points) { std::swap(points_ ,points); };
		void SetPoints(std::shared_ptr<o3d_PointCloud> points) {
			points_ = points;
		};
		std::shared_ptr<o3d_PointCloud> GetPoints() { return points_; }
	};
}
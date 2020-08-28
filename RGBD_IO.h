#pragma once
#include <memory>
#include <Open3D/Open3D.h>
#include <k4a/k4a.h>
#include "VTPLibInterface.h"

namespace Reco3D
{
	typedef Eigen::Matrix4d ImagePose;
	typedef open3d::geometry::RGBDImage ImageRGBD;

	// Struct to encapsulate pose transform and image data
	struct RGBDCapture_t
	{
		ImagePose pose_;
		ImageRGBD image_;
	};

	namespace IO
	{
		// Configuration class for RGBDSensor class
		class RGBDSensor_Config {};

		// Configuration class for Kinect/Vive trackers
		class RGBDSensor_Config_KinectVive : public RGBDSensor_Config
		{
		public:
			open3d::io::AzureKinectSensorConfig kinectConfig_;
		};

		// Virtual class for handling the hardware
		// i.e. initializing sensors and getting RGBD Data
		class RGBDSensor
		{
		public:
			virtual std::shared_ptr<RGBDCapture_t> CaptureFrame() = 0;
		};


		// Concrete class which takes RGBD images from Azure Kinect and 
		// positional data from Vive Trackers
		class RGBDSensor_KinectVive : public RGBDSensor
		{
		public:
			RGBDSensor_KinectVive(RGBDSensor_Config_KinectVive config);
			~RGBDSensor_KinectVive();
			std::shared_ptr<RGBDCapture_t> CaptureFrame() override;
		protected:
			RGBDSensor_Config_KinectVive config_;
			std::unique_ptr<open3d::io::AzureKinectSensor> sensor_;
			std::unique_ptr<VTPLibInterface> vtplib_;
			bool InitializeAzureKinect();
			bool InitializeVTPLib();
		};
	}
}
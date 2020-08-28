#pragma once
#include <memory>
#include <Open3D/Open3D.h>
#include <k4a/k4a.h>
#include "VTPLibInterface.h"
#include "Types.h"

namespace Reco3D
{
//	typedef Eigen::Matrix4d ImagePose;
//	typedef std::shared_ptr<open3d::geometry::RGBDImage> ImageRGBD;

	// Struct to encapsulate pose transform and image data
//	struct RGBDCapture_t
//	{
//		RGBDCapture_t() {};
//		ImagePose pose_;
//		ImageRGBD image_;
//	};

	namespace IO
	{
		// Configuration class for RGBDSensor class
		class RGBDSensor_Config {};

		// Configuration class for Kinect/Vive trackers
		struct RGBDSensor_Config_KinectVive
		{
			RGBDSensor_Config_KinectVive() :
				enableAlignDepthWithColor_(true),
				sensorIndex_(0),
				trackerIndex_(0)
			{}

			// Kinect
			const open3d::io::AzureKinectSensorConfig kinectConfig_;
			int sensorIndex_; // Kinect sensor index
			bool enableAlignDepthWithColor_;
			// Vive Tracker
			TrackerId trackerIndex_; // Default device index
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
			std::unique_ptr<VTPLibInterface> vtpInterface_;
			TrackerId currentTrackerIndex_;
			bool InitializeAzureKinect();
			bool InitializeVTPLib();
		};
	}
}
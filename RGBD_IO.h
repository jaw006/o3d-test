#pragma once
#define _USE_MATH_DEFINES
#include <memory>
#include <math.h>
#include <Open3D/Open3D.h>
#include <k4a/k4a.h>
#include "VTPLibInterface.h"
#include "Types.h"

namespace Reco3D
{
	namespace IO
	{
		// Configuration class for RGBDSensor class
		class RGBDSensor_Config {};

		// Configuration class for Kinect/Vive trackers
		class RGBDSensor_Config_KinectVive : public RGBDSensor_Config
		{
		public:
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
            bool SaveFrameToDisk();
			std::shared_ptr<RGBDCapture_t> CaptureFrame() override;
			ImagePose GetTrackerPose();
			CameraPoseParameters ConstructCameraPose(ImagePose& imgPose);
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
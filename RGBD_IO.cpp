#include "RGBD_IO.h"

Reco3D::IO::RGBDSensor_KinectVive::RGBDSensor_KinectVive(RGBDSensor_Config_KinectVive config) :
    sensor_(nullptr),
    vtpInterface_(nullptr),
    config_(config),
    currentTrackerIndex_(0)
{
    if (!InitializeAzureKinect() || !InitializeVTPLib())
        std::cerr << "Error initializing RGBDSensor_KinectVive!" << std::endl;
    else
        std::cerr << "Successfully initialized RGBDSensor_KinectVive!" << std::endl;
}

Reco3D::IO::RGBDSensor_KinectVive::~RGBDSensor_KinectVive()
{
}

std::shared_ptr<Reco3D::RGBDCapture_t> Reco3D::IO::RGBDSensor_KinectVive::CaptureFrame()
{
    return std::shared_ptr<RGBDCapture_t>();
}

bool Reco3D::IO::RGBDSensor_KinectVive::InitializeAzureKinect()
{
    using namespace open3d::io;
//    sensor_ = std::unique_ptr<AzureKinectSensor>(new AzureKinectSensor(config_.kinectConfig_));
    sensor_ = std::make_unique<AzureKinectSensor>(config_.kinectConfig_);
    if (!sensor_->Connect(config_.sensorIndex_)) {
        return false;
    }
    return true;
}

bool Reco3D::IO::RGBDSensor_KinectVive::InitializeVTPLib()
{
    vtpInterface_ = std::make_unique<VTPLibInterface>();

    auto ids = vtpInterface_->GetTrackerIds();
    if (ids.empty())
    {
        std::cout << "Error: No VIVE Trackers found!";
        return false;
    }
    else
    {
        std::cout << "Found devices with ids:(";
        for(auto id : ids)
        {
            std::cout << id << " ";
        }
        std::cout << std::endl;
        // If preferredTrackerId returns identity matrix, use the first other tracker found for position
        if (!vtpInterface_->GetTrackerMatrix4d(config_.trackerIndex_).isIdentity())
        {
            currentTrackerIndex_ = config_.trackerIndex_;
        }
        else
        {
            currentTrackerIndex_ = ids[0];
        }
    }
    std::cout << "Using tracker id# " << currentTrackerIndex_ << "for pose" << std::endl;
    return true;
}


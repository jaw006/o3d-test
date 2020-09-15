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
    std::shared_ptr<Reco3D::RGBDCapture_t> capture(new Reco3D::RGBDCapture_t());
    // Get Kinect image
    if (sensor_)
    {
        auto im_rgbd = sensor_->CaptureFrame(config_.enableAlignDepthWithColor_);
        if (im_rgbd != nullptr)
        {
            capture->image_ = im_rgbd;
        }
    }
    // Get tracker matrix 
    if (vtpInterface_)
    {
//        capture->pose_ = vtpInterface_->GetTrackerMatrix4d(currentTrackerIndex_);
        capture->pose_ = GetTrackerPose();
        capture->quat_ = vtpInterface_->GetTrackerQuaternion(currentTrackerIndex_);
    }
    return capture;
}

Reco3D::ImagePose Reco3D::IO::RGBDSensor_KinectVive::GetTrackerPose()
{
    if (vtpInterface_)
    {
        Reco3D::ImagePose pose = vtpInterface_->GetTrackerMatrix4d(currentTrackerIndex_);
        // Transformation from Tracker to Camera coords
        const Eigen::Affine3d aff = 
                    Eigen::Affine3d::Identity() *
                    Eigen::AngleAxisd(M_PI,     Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(M_PI/2,   Eigen::Vector3d::UnitX());
        pose *= aff.matrix();
        return pose;
    }

    return ImagePose();
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
//        aff.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    {
        std::cout << "Error: No VIVE Trackers found!";
        return false;
    }
    else
    {
        std::cout << "Found devices with ids: ";
        for(auto id : ids)
        {
            std::cout << id << " ";
        }
        std::cout << std::endl;
        currentTrackerIndex_ = ids[0];
        bool id_set = false;
        for(auto id : ids)
        {
            if (!id_set && !vtpInterface_->GetTrackerMatrix4d(id).isIdentity())
            {
                currentTrackerIndex_ = id;
                id_set = true;
            }
        }
//        // If preferredTrackerId returns identity matrix, use the first other tracker found for position
//        if (!vtpInterface_->GetTrackerMatrix4d(config_.trackerIndex_).isIdentity())
//        {
//            currentTrackerIndex_ = config_.trackerIndex_;
//        }
//        else
//        {
//            currentTrackerIndex_ = ids[0];
//        }
    }
    std::cout << "Using tracker id# " << currentTrackerIndex_ << " for pose" << std::endl;
    if (currentTrackerIndex_ == 0)
    {
        std::cout << "WARNING USING HMD FOR POSE" << std::endl;
        std::cout << "WARNING USING HMD FOR POSE" << std::endl;
        std::cout << "WARNING USING HMD FOR POSE" << std::endl;
    }
    return true;
}


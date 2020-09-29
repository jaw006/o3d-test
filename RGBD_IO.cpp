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

bool Reco3D::IO::RGBDSensor_KinectVive::SaveFrameToDisk()
{
    std::shared_ptr<Reco3D::RGBDCapture_t> capture(new Reco3D::RGBDCapture_t());
    if (sensor_)
    {
        auto im_rgbd = sensor_->CaptureFrame(config_.enableAlignDepthWithColor_);
        if (im_rgbd != nullptr)
        {
            time_t t = time(0);
            std::string filename = "data/" + std::to_string(t);
            std::string ext = ".png";
            const int quality = 100;
            open3d::geometry::Image& color = im_rgbd->color_;
            open3d::geometry::Image& depth = im_rgbd->depth_;
            std::shared_ptr<open3d::geometry::Image> depthFloat = depth.open3d::geometry::Image::ConvertDepthToFloatImage();
            
            std::cout << "Writing capture to disk!" << std::endl;
            open3d::io::WriteImageToPNG(filename + "_color" + ext, color, quality);
            open3d::io::WriteImageToPNG(filename + "_depth" + ext, *depthFloat, quality);
            return true;
        }
        return false;
    }
    else
    {
        std::cout << "Sensor not connected!" << std::endl;
        return false;
    }

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
        Reco3D::ImagePose pose = GetTrackerPose();
        capture->pose_ = pose;
        capture->quat_ = vtpInterface_->GetTrackerQuaternion(currentTrackerIndex_);
        capture->camPose_ = ConstructCameraPose(pose);
    }
    return capture;
}

Reco3D::ImagePose Reco3D::IO::RGBDSensor_KinectVive::GetTrackerPose()
{
    if (vtpInterface_)
    {
        Reco3D::ImagePose pose = vtpInterface_->GetTrackerMatrix4d(currentTrackerIndex_);
        Eigen::Vector3d posePosition = { pose(0,3), pose(1,3), pose(2,3) };
        Eigen::Matrix4d posePositionMatrix = Eigen::Matrix4d::Identity();
        posePositionMatrix(0, 3) = pose(0, 3);
        posePositionMatrix(1, 3) = pose(1, 3);
        posePositionMatrix(2, 3) = pose(2, 3);
        // Transformation from Tracker to Camera coords
        const Eigen::Affine3d aff =
            Eigen::Affine3d::Identity() *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
//                    Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX());
        pose *= aff.matrix();
        return pose;
    }

    return ImagePose();
}

// imgPose - extrinsic matrix
Reco3D::CameraPoseParameters Reco3D::IO::RGBDSensor_KinectVive::ConstructCameraPose(Reco3D::ImagePose& imgPose)
{
    Reco3D::CameraPoseParameters camPoseParams;

    const open3d::camera::PinholeCameraIntrinsic intrinsic(
        CAMERA_RES_X, 
        CAMERA_RES_Y,
        INTRINSIC_FX,
        INTRINSIC_FY,
        INTRINSIC_CX,
        INTRINSIC_CY);
    Eigen::Matrix4d_u extrinsic(imgPose);

    camPoseParams.intrinsic_ = intrinsic;
    camPoseParams.extrinsic_ = extrinsic;
    return camPoseParams;
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

        // This is dumb
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


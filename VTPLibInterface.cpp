#include "VTPLibInterface.h"

Reco3D::VTPLibInterface::VTPLibInterface() :
    system_(nullptr)
{
    system_ = new vtp::System();
}

Reco3D::VTPLibInterface::~VTPLibInterface()
{
    delete system_;
}

std::vector<Reco3D::TrackerId> Reco3D::VTPLibInterface::GetTrackerIds()
{
    std::vector<TrackerId> ids;
    if (system_ != nullptr)
    {
        std::vector<vtp::Device> trackedDevices = system_->GetTrackedDevices();

        for (auto device : trackedDevices)
        {
            // Find ids of trackers with model name matching VIVE_STR
            if (device.modelName_.find(VIVE_STR, 0) != std::string::npos)
            {
                ids.push_back(device.GetDeviceId());
            }
        }
    }
    return ids;
}

Eigen::Matrix4d Reco3D::VTPLibInterface::GetTrackerMatrix4d(TrackerId& deviceId)
{
    auto device = system_->GetDevice(deviceId);
    if (device.IsValid() && device.GetDeviceClass() != vtp::DeviceClass::Invalid)
    {
        vtp::Pose_t& pose = device.GetPose();
        Eigen::MatrixBase<Eigen::Matrix4d> m(vtpPoseToEigenMatrix4d(pose));
        return m;
    }
    // Invalid matrix
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d Reco3D::VTPLibInterface::vtpPoseToEigenMatrix4d(vtp::Pose_t& pose)
{
    vtp::Vector3_t& pos = pose.Position;
    vtp::Matrix3_t& rot = pose.Rotation;
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    //Copy rotation
    for (int x = 0; x < 3; x++)
        for (int y = 0; x < 3; x++)
            m(x, y) = rot.m[x][y];
    // Copy position
    m(3, 0) = pos.v[0];
    m(3, 1) = pos.v[1];
    m(3, 2) = pos.v[2];
    m(3, 3) = 1.0;
    return m;
}

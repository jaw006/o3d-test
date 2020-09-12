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
//        //DEBUG
//        std::cout << "Pose.Position=" << pose.Position.toString() << std::endl;
//        std::cout << "Pose.Rotation=" << std::endl << pose.Rotation.toString() << std::endl;
        Eigen::Matrix4d m(vtpPoseToEigenMatrix4d(pose));
//        std::cout << "ConvertedPose=" << std::endl << m << std::endl;
        return m;
    }
    // Invalid matrix
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d Reco3D::VTPLibInterface::vtpPoseToEigenMatrix4d(vtp::Pose_t& pose)
{
    vtp::Vector3_t& pos = pose.Position;
    vtp::Matrix3_t& rot = pose.Rotation;
    Eigen::Matrix4d m;
//    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    //Copy rotation
//    for (int x = 0; x < 3; x++)
//        for (int y = 0; y < 3; y++)
//            m(x, y) = (double)rot.m[x][y];
    m(0,0) = (double)rot.m[0][0];
    m(0,1) = (double)rot.m[1][0];
    m(0,2) = (double)rot.m[2][0];
    m(0,3) = (double)pos.v[0];
    m(1,0) = (double)rot.m[0][1];
    m(1,1) = (double)rot.m[1][1];
    m(1,2) = (double)rot.m[2][1];
    m(1,3) = (double)pos.v[1];
    m(2,0) = (double)rot.m[0][2];
    m(2,1) = (double)rot.m[1][2];
    m(2,2) = (double)rot.m[2][2];
    m(2,3) = (double)pos.v[2];
    // Copy position
    m(3, 0) = 0.0;
    m(3, 1) = 0.0;
    m(3, 2) = 0.0;
    m(3, 3) = 1.0;
    return m;
}

Eigen::Vector3d Reco3D::VTPLibInterface::GetTrackerPosition(TrackerId& deviceId)
{
    auto p = GetPose(deviceId).Position;
    return Eigen::Vector3d(
        (double)p.v[0],
        (double)p.v[1],
        (double)p.v[2]
    );
}

Eigen::Matrix3d Reco3D::VTPLibInterface::GetTrackerRotation(TrackerId& deviceId)
{
    auto r = GetPose(deviceId).Rotation;
    Eigen::Matrix3d m;
//    m << r.m[0][0];     
//    m << r.m[1][0]; 
//    m << r.m[2][0];
//    m << r.m[0][1];     
//    m << r.m[1][1]; 
//    m << r.m[2][1];
//    m << r.m[0][2];     
//    m << r.m[1][2]; 
//    m << r.m[2][2];
    m << r.m[0][0];     
    m << r.m[0][1]; 
    m << r.m[0][2];
    m << r.m[1][0];     
    m << r.m[1][1]; 
    m << r.m[1][2];
    m << r.m[2][0];     
    m << r.m[2][1]; 
    m << r.m[2][2];
    return m;
}

Eigen::Vector4d Reco3D::VTPLibInterface::GetTrackerQuaternion(TrackerId& deviceId)
{
    auto q = GetPose(deviceId).Quaternion;
    Eigen::Vector4d quat;
    quat << q.w, q.x, q.y, q.z;
    return quat;
}

void Reco3D::VTPLibInterface::testVTPLibInterface()
{
    vtp::Pose_t pose;
    vtp::Matrix3_t& rot = pose.Rotation;
    vtp::Vector3_t& pos = pose.Position;

    pos.v[0] = pos.v[1] = pos.v[2] = 1.0f;
    rot.m[0][0] = rot.m[0][1] = rot.m[0][2] = 0.0f;
    rot.m[1][0] = rot.m[1][1] = rot.m[1][2] = 3.0f;
    rot.m[2][0] = rot.m[2][1] = rot.m[2][2] = 6.0f;

    std::cerr << "Test: " << vtpPoseToEigenMatrix4d(pose)<< std::endl;
    std::cerr << "End tests ---------" << std::endl;
}

vtp::Pose_t Reco3D::VTPLibInterface::GetPose(TrackerId& deviceId)
{
    auto device = system_->GetDevice(deviceId);
    return device.GetPose();
}

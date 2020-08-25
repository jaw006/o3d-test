#pragma once
#include <Eigen/Core>
#include "thirdparty/ViveTrackerPosition/vtplib/include/vtp_system.h"

#define VIVE_STR "VIVE Tracker"

namespace Reco3D
{
    typedef uint32_t TrackerId;
    class VTPLibInterface
    {
    private:
        vtp::System* system_;
    public:
        VTPLibInterface();
        ~VTPLibInterface();
        std::vector<TrackerId> GetTrackerIds();
        Eigen::Matrix4d GetTrackerMatrix4d(TrackerId& deviceId);
        Eigen::Matrix4d vtpPoseToEigenMatrix4d(vtp::Pose_t& pose);
    };
}
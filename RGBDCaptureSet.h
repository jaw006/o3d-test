#pragma once
#include <vector>
#include "PointCloud.h"
#include "PointsToMesh.h"
#include "RGBDToPoints.h"
#include "Types.h"

namespace Reco3D
{
    // Takes a capture, converts it to points
    // and stores the point cloud representations
    class RGBDCaptureSet
    {
    private:
        std::vector<PointCloud>    points_;
        std::vector<RGBDCapture_t> captures_;

        std::shared_ptr<RGBDToPoints>   rgbdToPoints_;
        std::shared_ptr<PointsToMesh>   pointsToMesh_;
        std::shared_ptr<o3d_PointCloud> combinedPoints_;
        std::shared_ptr<o3d_TriMesh>    combinedMesh_;

    public:
        RGBDCaptureSet();
        ~RGBDCaptureSet();
        void AddCapture(std::shared_ptr<RGBDCapture_t> capture);
        std::shared_ptr<o3d_PointCloud> GetCombinedPointCloud();
        std::shared_ptr<o3d_TriMesh> GetCombinedTriangleMesh();
    };
}
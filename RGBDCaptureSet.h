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
        std::vector<RGBDCapture_t> captures_;
        std::vector<std::shared_ptr<PointCloud>>    points_;

        std::shared_ptr<RGBDToPoints>   rgbdToPoints_;
        std::shared_ptr<PointsToMesh>   pointsToMesh_;

        std::shared_ptr<o3d_PointCloud> combinedPoints_;
        std::shared_ptr<o3d_TriMesh>    combinedMesh_;

        std::shared_ptr<PointsVector>   pointsVector_;

    public:
        RGBDCaptureSet();
        ~RGBDCaptureSet();
        void AddCapture(std::shared_ptr<RGBDCapture_t> capture);
        size_t Count();
        std::shared_ptr<PointCloud> GetCombinedPointCloud();
        std::shared_ptr<o3d_TriMesh> GetCombinedTriangleMesh();
    };
}
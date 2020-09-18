#pragma once
#include <vector>
#include "PointCloud.h"
#include "PointsToMesh.h"
#include "RGBDToPoints.h"
#include "ColorMap.h"
#include "Types.h"

namespace Reco3D
{
    // Takes a capture, converts it to points
    // and stores the point cloud representations
    class RGBDCaptureSet
    {
    private:
        std::vector<std::shared_ptr<RGBDCapture_t>> captures_;
        std::vector<std::shared_ptr<PointCloud>>    points_;

        std::shared_ptr<RGBDToPoints>   rgbdToPoints_;
        std::shared_ptr<PointsToMesh>   pointsToMesh_;

        std::shared_ptr<o3d_PointCloud> combinedPoints_;
        std::shared_ptr<o3d_TriMesh>    combinedMesh_;
        std::shared_ptr<PointsVector>   pointsVector_;

        std::shared_ptr<ColorMap>   colorMap_;

        std::shared_ptr<PointCloud> GetSourcePointCloud();

    public:
        RGBDCaptureSet();
        ~RGBDCaptureSet();
        void AddCapture(std::shared_ptr<RGBDCapture_t> capture);
//        void AddCloud(std::shared_ptr<PointCloud> cloud); // TODO: For loading in saved clouds
        void Clear();
        size_t Count();
        std::shared_ptr<PointCloud> GetCombinedPointCloud();
        const std::vector<std::shared_ptr<Reco3D::PointCloud>>& GetPointsVector();
        std::shared_ptr<o3d_TriMesh> GetCombinedTriangleMesh();
    protected:
        void ClampMaxPointsSize();
        size_t SumPoints();
    };
}
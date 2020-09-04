#pragma once

#include <memory>
#include <Open3D/Open3D.h>
#include "Types.h"
#include "PointCloud.h"

namespace Reco3D {
    // Vector of multiple points objects
    // Used for combining multiple points objects
    class PointsVector
    {
    public:
        PointsVector();
        ~PointsVector();

        std::shared_ptr<Reco3D::PointCloud> GetCombinedPoints();
        bool AddPoints(std::shared_ptr<Reco3D::PointCloud> points);
        size_t Count();

    private:
        std::vector<std::shared_ptr<Reco3D::PointCloud>> pointsVector_;
        std::shared_ptr<Reco3D::PointCloud> combinedPoints_;
    };

    class PointsToMesh
    {
    public:
        PointsToMesh();
        ~PointsToMesh();

        std::shared_ptr<o3d_TriMesh> ToMesh(std::shared_ptr<Reco3D::PointsVector>& pointsVector);

        struct MeshConfig_t
        {
            size_t depth;
            size_t width;
            float scale;
            bool linear_fit;
        };

    private:
    };

}
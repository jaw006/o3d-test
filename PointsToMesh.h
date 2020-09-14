#pragma once

#define _USE_MATH_DEFINES

#include <memory>
#include <math.h>
#include <Open3D/Open3D.h>
#include <Eigen/Geometry>
#include "Types.h"
#include "PointCloud.h"

namespace Reco3D {
    // Vector of multiple points objects
    // Used for combining multiple points objects
    class PointsVector
    {
    private: 
        std::vector<std::shared_ptr<Reco3D::PointCloud>> pointsVector_;
        std::shared_ptr<Reco3D::PointCloud> combinedPoints_;
        ImagePose sourcePose;
        ImagePose sourcePoseInverse;
    public:
        PointsVector();
        ~PointsVector();

        bool AddPoints(std::shared_ptr<Reco3D::PointCloud> points);
        size_t Count();

        std::shared_ptr<Reco3D::PointCloud> GetCombinedPoints();
        const std::vector<std::shared_ptr<Reco3D::PointCloud>>& GetPointsVector() { return pointsVector_; };
        std::shared_ptr<Reco3D::PointCloud> GetSourcePointCloud();
        open3d::registration::RegistrationResult RegisterPoints(std::shared_ptr<Reco3D::PointCloud> source, std::shared_ptr<Reco3D::PointCloud> target);

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
#pragma once

#define _USE_MATH_DEFINES

#include <memory>
#include <math.h>
#include <Open3D/Open3D.h>
#include <Eigen/Geometry>
#include "Types.h"
#include "PointCloud.h"

#define DOWNSAMPLE_VOXEL_SIZE 0.01
#define MAX_POINT_COUNT 100000
#define MIN_POINT_COUNT 1000
#define MAX_CAPTURES 10000

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
        void RegisterPoints(std::shared_ptr<Reco3D::PointCloud>& points);
        open3d::registration::RegistrationResult EvaluateCurrentRegistration(std::shared_ptr<Reco3D::PointCloud>& source, std::shared_ptr<Reco3D::PointCloud>& target, Eigen::Matrix4d& m);
        size_t Count();

        std::shared_ptr<Reco3D::PointCloud> GetCombinedPoints();
        size_t SumPoints();
        void ClampMaxPointsSize();
        const std::vector<std::shared_ptr<Reco3D::PointCloud>>& GetPointsVector() { return pointsVector_; };
        std::shared_ptr<Reco3D::PointCloud> GetSourcePointCloud();
        open3d::registration::RegistrationResult RegisterPointsICP(std::shared_ptr<Reco3D::PointCloud> source, std::shared_ptr<Reco3D::PointCloud> target, Reco3D::ImagePose& target_pose);
//        open3d::registration::RegistrationResult RegisterPoints(std::shared_ptr<Reco3D::PointCloud> source, std::shared_ptr<Reco3D::PointCloud> target);

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
#include "PointsToMesh.h"

Reco3D::PointsToMesh::PointsToMesh()
{
}

Reco3D::PointsToMesh::~PointsToMesh()
{
}

std::shared_ptr<Reco3D::o3d_TriMesh> Reco3D::PointsToMesh::ToMesh(std::shared_ptr<Reco3D::PointsVector>& pointsVector)
{
    // Empty points vector
    if (pointsVector->Count() <= 0)
    {
        return std::make_shared<Reco3D::o3d_TriMesh>();
    }

    Reco3D::o3d_PointCloud cloud = *pointsVector->GetCombinedPoints()->GetPoints();

    // Downsample before calculating normals
    cloud.UniformDownSample(100.0);
    
    // Point cloud must have normals to mesh
    if (!cloud.HasNormals())
    {
        bool normalsGenerated = cloud.EstimateNormals();
    }

//    // Mesh with default parameters - Poisson reconstruction
    // 12 is good
    // 8 is faster
    uint64_t depth = 4;
   std::tuple<std::shared_ptr<o3d_TriMesh>, std::vector<double>> tuple_result =
       open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(cloud,depth);

   std::shared_ptr<o3d_TriMesh> output = std::get<std::shared_ptr<o3d_TriMesh>>(tuple_result);

   output->FilterSmoothSimple(2);

    return output;
}

Reco3D::PointsVector::PointsVector() :
    combinedPoints_(std::shared_ptr<Reco3D::PointCloud>(new Reco3D::PointCloud()))
{
}

Reco3D::PointsVector::~PointsVector()
{
}

std::shared_ptr<Reco3D::PointCloud> Reco3D::PointsVector::GetCombinedPoints()
{
    if (Count() > 0)
    {
        auto sourcePointCloud = pointsVector_.at(0);
        if (sourcePointCloud != nullptr)
        {
            // Set pose from the first point cloud in vector 
            const ImagePose& sourcePose = sourcePointCloud->GetPose();
            combinedPoints_->SetPose(sourcePose);
        }
    }
    return combinedPoints_;
}

bool Reco3D::PointsVector::AddPoints(std::shared_ptr<Reco3D::PointCloud> points)
{
    // TODO: Check for duplicates, return false if not added
    if (points == nullptr)
        return false;

    // Add points together
    pointsVector_.push_back(points);
    std::shared_ptr<Reco3D::o3d_PointCloud> addedPts(new Reco3D::o3d_PointCloud());
    *addedPts = *combinedPoints_->GetPoints() + *points->GetPoints();
    combinedPoints_->SetPoints(addedPts);
    return true;
}

size_t Reco3D::PointsVector::Count()
{
    return pointsVector_.size();
}
#include "PointsToMesh.h"

Reco3D::PointsToMesh::PointsToMesh()
{
}

Reco3D::PointsToMesh::~PointsToMesh()
{
}

std::shared_ptr<Reco3D::o3d_TriMesh> Reco3D::PointsToMesh::ToMesh(std::shared_ptr<Reco3D::PointsVector>& pointsVector)
{
    double quantile = 0.02; // Points with densities below this threshold will be removed

    // Empty points vector
    if (pointsVector->Count() <= 0)
    {
        return std::make_shared<Reco3D::o3d_TriMesh>();
    }

    Reco3D::o3d_PointCloud cloud = *pointsVector->GetCombinedPoints()->GetPoints();

    // Downsample before calculating normals
//    cloud = *cloud.UniformDownSample(2);
    cloud = *cloud.VoxelDownSample(DOWNSAMPLE_VOXEL_SIZE);
    
    // Point cloud must have normals to mesh
    if (!cloud.HasNormals())
    {
        bool normalsGenerated = cloud.EstimateNormals();
    }

    //      Mesh reconstruction - Ball Pivoting
    //      Don't use this - it produces worse results on every run
    //    const std::vector<double> radii{ 0.01, 0.02, 0.04 };
    //    std::shared_ptr<open3d::geometry::TriangleMesh> output(open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(cloud, radii));

    // Mesh with default parameters - Poisson reconstruction
    // 12 is good, 8 is faster
    uint64_t depth = 16;
    std::tuple<std::shared_ptr<o3d_TriMesh>, std::vector<double>> tuple_result =
             open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(cloud,depth);
    std::shared_ptr<o3d_TriMesh> output = std::get<std::shared_ptr<o3d_TriMesh>>(tuple_result);
    std::vector<double> densities = std::get<std::vector<double>>(tuple_result);

    // Remove vertices created that fall below 0.01 quantile
    // http://www.open3d.org/docs/release/tutorial/Advanced/surface_reconstruction.html
    // https://www.statisticshowto.com/quantile-definition-find-easy-steps/
    std::vector<double> densities_sorted = densities;
    std::sort(densities_sorted.begin(), densities_sorted.end());
    int index = (int) std::floor(quantile * (densities_sorted.size() + 1));
    double threshold = densities_sorted.at(index);

    // Add indices of vertices to remove to this vector
    std::vector<size_t> indices_to_remove;
    for (size_t index = 0; index < densities.size(); index++)
    {
        if (densities.at(index) < threshold)
        {
            indices_to_remove.push_back(index);
        }
    }
    std::cout << "Removing " << indices_to_remove.size() << " vertices!" << std::endl;
    output->RemoveVerticesByIndex(indices_to_remove);

    // Decimate
   size_t numTriangles = output->triangles_.size();
   size_t decimationFactor = 10;
   size_t numTrianglesDecimated = numTriangles / decimationFactor;
   std::cout << "Decimating " << numTriangles << " into " << numTrianglesDecimated << " triangles." << std::endl;
   output->SimplifyQuadricDecimation((int)numTrianglesDecimated);

   // Post processing
//   output->FilterSmoothSimple(2);

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
    std::shared_ptr<Reco3D::o3d_PointCloud> addedPts(new Reco3D::o3d_PointCloud());
    for (auto points : pointsVector_)
    {
        *addedPts = *addedPts + *points->GetPoints();
    }
    combinedPoints_->SetPoints(addedPts);
    return combinedPoints_;
}

// Adds points to vector
// https://stackoverflow.com/questions/58727178/having-trouble-aligning-2d-lidar-pointcloud-to-match-the-coordinate-system-of-ht
// https://math.stackexchange.com/questions/1234948/inverse-of-a-rigid-transformation
// http://www.graphics.stanford.edu/courses/cs248-98-fall/Final/q4.html
// Notes : Construct inverse by transposing rotation and negating transform
//          Camera forward is +Z
bool Reco3D::PointsVector::AddPoints(std::shared_ptr<Reco3D::PointCloud> points)
{
    // TODO: Check for duplicates, return false if not added
    if (points == nullptr)
         return false;

    // Downsample before doing transfomrations
    points->SetPoints(points->GetPoints()->VoxelDownSample(DOWNSAMPLE_VOXEL_SIZE));

    // +Z corresponds to -Y
    // Calculate matrices 
    ImagePose pose = points->GetPose();
    ImageQuaternion quat = points->GetQuat();
    Eigen::Vector3d posePosition = { pose(0,3), pose(1,3), pose(2,3) };
    Eigen::Matrix4d posePositionMatrix = Eigen::Matrix4d::Identity();
    posePositionMatrix(0, 3) = pose(0, 3);
    posePositionMatrix(1, 3) = pose(1, 3);
    posePositionMatrix(2, 3) = pose(2, 3);
    Eigen::Matrix3d quatRotation = open3d::geometry::Geometry3D::GetRotationMatrixFromQuaternion(quat);

    // Transform points by rotating 180 on Z axis and -90 on Y axis
    const Eigen::Affine3d aff = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX());
    points->GetPoints()->Transform(aff.matrix());
    points->GetPoints()->Transform(posePositionMatrix);
    points->GetPoints()->Rotate(quatRotation, posePosition);

    // Registration
    // Not sure if points get transformed when passed
//    if (Count() > 0)
//    {
//        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
//        auto reg_result = RegisterPoints(GetSourcePointCloud(), points, m);
//        points->GetPoints()->Transform(reg_result.transformation_);
//    }

    // Add to vector
    pointsVector_.push_back(points);
    return true;
}

size_t Reco3D::PointsVector::Count()
{
    return pointsVector_.size();
}

std::shared_ptr<Reco3D::PointCloud> Reco3D::PointsVector::GetSourcePointCloud()
{
    if (Count() == 0)
    {
        return nullptr;
    }
    else
    {
        return pointsVector_.at(0);
    }
}

open3d::registration::RegistrationResult Reco3D::PointsVector::RegisterPoints(std::shared_ptr<Reco3D::PointCloud> source, 
    std::shared_ptr<Reco3D::PointCloud> target, Reco3D::ImagePose& target_pose)
{

     auto estimation = open3d::registration::TransformationEstimationPointToPoint(true);
     auto criteria = open3d::registration::ICPConvergenceCriteria();
     double max_correspondence_distance = 1.0;
     criteria.max_iteration_ = 30;
     auto reg_result = open3d::registration::RegistrationICP(
         *source->GetPoints(),
         *target->GetPoints(),
         max_correspondence_distance,
         target_pose,
         estimation,
         criteria
     );
    // Print fitness, RMSE
    double& fitness = reg_result.fitness_; 
    double& rmse = reg_result.inlier_rmse_;
    std::string log1 = "Fitness= " + std::to_string(fitness) + "\n";
    std::string log2 = "RMSE= " + std::to_string(rmse) + "\n";
    std::cout << "Transformation Estimation:\n" << reg_result.transformation_ << std::endl;
    open3d::utility::LogInfo(log1.c_str());
    open3d::utility::LogInfo(log2.c_str());
    return reg_result;
}

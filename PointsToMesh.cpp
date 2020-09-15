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
//    cloud = *cloud.UniformDownSample(2);
    cloud = *cloud.VoxelDownSample(DOWNSAMPLE_VOXEL_SIZE);
    
    // Point cloud must have normals to mesh
    if (!cloud.HasNormals())
    {
        bool normalsGenerated = cloud.EstimateNormals();
    }

//    // Mesh reconstruction - Ball Pivoting
//    const std::vector<double> radii{ 0.01, 0.02, 0.04 };
//    std::shared_ptr<open3d::geometry::TriangleMesh> output(open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(cloud, radii));

//    // Mesh with default parameters - Poisson reconstruction
    // 12 is good
    // 8 is faster
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
    double quantile = 0.01;
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
   output->SimplifyQuadricDecimation(numTrianglesDecimated);

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
//    if (Count() > 0)
//    {
//        auto sourcePointCloud = pointsVector_.at(0);
//        if (sourcePointCloud != nullptr)
//        {
//            // Set pose from the first point cloud in vector 
//            const ImagePose& sourcePose = sourcePointCloud->GetPose();
//            combinedPoints_->SetPose(sourcePose);
//        }
//    }
    return combinedPoints_;
}

bool Reco3D::PointsVector::AddPoints(std::shared_ptr<Reco3D::PointCloud> points)
{
    // TODO: Check for duplicates, return false if not added
    if (points == nullptr)
         return false;

    // Downsample before doing transfomrations
    points->SetPoints(points->GetPoints()->VoxelDownSample(DOWNSAMPLE_VOXEL_SIZE));

//    // Transform to shared coord system
    if (Count() == 0)
    {
//        // Source transformation
        sourcePose = points->GetPose();
        sourcePoseInverse = points->GetPose().inverse();
//       points->GetPoints()->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
    }
    else
    {
//        points->GetPoints()->PaintUniformColor(Eigen::Vector3d(1.0, 1.0-(Count()*0.2), 1.0-(Count()*0.1)));
//        auto reg_result = RegisterPoints(GetSourcePointCloud(), points);
//        points->GetPoints()->Transform(reg_result.transformation_);
//
    }
    // https://stackoverflow.com/questions/58727178/having-trouble-aligning-2d-lidar-pointcloud-to-match-the-coordinate-system-of-ht
    // Construct inverse by transposing rotation and negating transform
    // Camera forward is +Z
    // https://math.stackexchange.com/questions/1234948/inverse-of-a-rigid-transformation
    // http://www.graphics.stanford.edu/courses/cs248-98-fall/Final/q4.html

    // +Z corresponds to -Y
    ImagePose pose = points->GetPose();
    ImageQuaternion quat = points->GetQuat();
    Eigen::Matrix3d quatRotation = open3d::geometry::Geometry3D::GetRotationMatrixFromQuaternion(quat);
    Eigen::Matrix3d quatRotationInverse = quatRotation;
    quatRotationInverse.transposeInPlace();

    Eigen::Matrix4d posePositionMatrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d inversePosePositionMatrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d posePosition;
    posePosition(0, 3) =  pose(0,3);
    posePosition(1, 3) =  pose(1,3);
    posePosition(2, 3) =  pose(2,3);
    Eigen::Vector3d inversePosePosition = -posePosition;
    posePositionMatrix(0, 3) = pose(0, 3);
    posePositionMatrix(1, 3) = pose(1, 3);
    posePositionMatrix(2, 3) = pose(2, 3);
    inversePosePositionMatrix(0, 3) = -pose(0, 3);
    inversePosePositionMatrix(1, 3) = -pose(1, 3);
    inversePosePositionMatrix(2, 3) = -pose(2, 3);

    ImagePose poseRotation = pose;
    poseRotation(0, 3) = 0.0;
    poseRotation(1, 3) = 0.0;
    poseRotation(2, 3) = 0.0;
    poseRotation(3, 0) = 0.0;
    poseRotation(3, 1) = 0.0;
    poseRotation(3, 2) = 0.0;
    poseRotation(3, 3) = 1.0;
    ImagePose inversePoseRotation = poseRotation;
    inversePoseRotation.transposeInPlace();

    // Copy untransformed points
//    std::shared_ptr<Reco3D::o3d_PointCloud> pointsCopy = std::make_shared<Reco3D::o3d_PointCloud>();
//    *pointsCopy += *points->GetPoints();
//    std::shared_ptr<Reco3D::PointCloud> copyPts = std::make_shared<Reco3D::PointCloud>(pointsCopy, points->GetCapture());
//    copyPts->GetPoints()->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
////    pointsCopy->GetPoints()->Transform(pose.inverse());
//    pointsVector_.push_back(copyPts);

//    std::cout << "Pose:\n" << pose << std::endl;
//    std::cout << "Quaternion:\n" << quat << std::endl;
//    std::cout << "Quaternion.w:\n" << std::to_string(quat(0)) << std::endl;
//    std::cout << "QuaternionRotation:\n" << quatRotation << std::endl;
//    std::cout << "posePositionMatrix:\n" << posePositionMatrix << std::endl;
//    std::cout << "InversePosePositionMatrix:\n" << inversePosePositionMatrix << std::endl;
//    std::cout << "Inverse:\n" << pose * inversePoseRotation * inversePosePositionMatrix << std::endl;


    // // Transform points by rotating 180 on Z axis and -90 on Y axisE?
    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
    aff.rotate(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX()));
    points->GetPoints()->Transform(aff.matrix());
    points->GetPoints()->Transform(posePositionMatrix);
    points->GetPoints()->Rotate(quatRotation, posePosition);
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
//    if (Count() > 0)
//    {
//        auto reg_result = RegisterPoints(GetSourcePointCloud(), points, m);
//        points->GetPoints()->Transform(reg_result.transformation_);
//    }
    pointsVector_.push_back(points);

//    std::shared_ptr<Reco3D::o3d_PointCloud> addedPts(new Reco3D::o3d_PointCloud());
//    *addedPts = *combinedPoints_->GetPoints() + *points->GetPoints();
//    combinedPoints_->SetPoints(addedPts);
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
     double max_correspondence_distance = 100.0;
     criteria.max_iteration_ = 100;
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

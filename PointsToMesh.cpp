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
    uint64_t depth = 12;
   std::tuple<std::shared_ptr<o3d_TriMesh>, std::vector<double>> tuple_result =
       open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(cloud,depth);

   std::shared_ptr<o3d_TriMesh> output = std::get<std::shared_ptr<o3d_TriMesh>>(tuple_result);

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
//    for (auto points : pointsVector_)
//    {
//        *addedPts = *addedPts + *points->GetPoints();
//    }
//    combinedPoints_->SetPoints(addedPts);
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

    // Downsample before calculating normals
    points->GetPoints()->UniformDownSample(1000.0);

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

    std::cout << "Pose:\n" << pose << std::endl;
    std::cout << "Quaternion:\n" << quat << std::endl;
    std::cout << "Quaternion.w:\n" << std::to_string(quat(0)) << std::endl;
    std::cout << "QuaternionRotation:\n" << quatRotation << std::endl;
    std::cout << "posePositionMatrix:\n" << posePositionMatrix << std::endl;
    std::cout << "InversePosePositionMatrix:\n" << inversePosePositionMatrix << std::endl;
    std::cout << "Inverse:\n" << pose * inversePoseRotation * inversePosePositionMatrix << std::endl;

//    Eigen::Affine3d t;
    // Rotate -90 in Z axis
//    t.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
//    points->GetPoints()->Transform(t.matrix());

//    std::cout << "Inverse Pose:\n" << inverse << std::endl;
//    std::cout << "InversePose * Pose:\n" << inverse*pose << std::endl;
//    std::cout << "Pose * InversePose:\n" << pose*inverse << std::endl;
//    Eigen::Matrix4d transformationMat = inversePosePositionMatrix * inversePoseRotation;

//    points->GetPoints()->Translate(posePosition, false);
    points->GetPoints()->Transform(pose.inverse());
    points->GetPoints()->Rotate(quatRotation, Eigen::Vector3d(0.0, 0.0, 0.0));
//    points->GetPoints()->Rotate(quatRotation, points->GetPoints()->GetCenter());

//    points->GetPoints()->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
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

open3d::registration::RegistrationResult Reco3D::PointsVector::RegisterPoints(std::shared_ptr<Reco3D::PointCloud> source, std::shared_ptr<Reco3D::PointCloud> target)
{

         auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
         auto criteria = open3d::registration::ICPConvergenceCriteria();
         double max_correspondence_distance = 10.0;
         criteria.max_iteration_ = 30;
         auto reg_result = open3d::registration::RegistrationICP(
             *source->GetPoints(),
             *target->GetPoints(),
             max_correspondence_distance,
             target->GetPose(),
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

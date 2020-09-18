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
   size_t decimationFactor = 2;
   size_t numTrianglesDecimated = numTriangles / decimationFactor;
   std::cout << "Decimating " << numTriangles << " into " << numTrianglesDecimated << " triangles." << std::endl;
   output->SimplifyQuadricDecimation((int)numTrianglesDecimated);

    if (!output->HasVertexNormals() || !output->HasTriangleNormals())
    {
        output = std::make_shared<o3d_TriMesh>(output->ComputeVertexNormals().ComputeTriangleNormals());
    }

   // Color Mapping

   // Post processing
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

Eigen::Matrix4d ExtractPoseTransform(Eigen::Matrix4d& m)
{
    Eigen::Matrix4d posePositionMatrix = Eigen::Matrix4d::Identity();
    posePositionMatrix(0, 3) = m(0, 3);
    posePositionMatrix(1, 3) = m(1, 3);
    posePositionMatrix(2, 3) = m(2, 3);
    return posePositionMatrix;
}

Eigen::Matrix4d ExtractPoseRotation(Eigen::Matrix4d& m)
{
    Eigen::Matrix4d rotMatrix = Eigen::Matrix4d::Identity();
    rotMatrix.topLeftCorner(3, 3) = m.topLeftCorner(3, 3);
    return rotMatrix;
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

size_t Reco3D::PointsVector::SumPoints()
{
    size_t sum = 0;
    for (auto cloudIt = pointsVector_.begin(); cloudIt != pointsVector_.end(); cloudIt++)
    {
        auto cloud = *cloudIt;
        if (cloud == nullptr)
        {
            pointsVector_.erase(cloudIt);
            continue;
        }
        sum += cloud->GetPoints()->points_.size();
    }
    std::cout << "Total point count: " << sum << std::endl;
    return sum;
}

void Reco3D::PointsVector::ClampMaxPointsSize()
{
    const size_t maxPoints = MAX_POINT_COUNT;
    const size_t minPoints = MIN_POINT_COUNT;
    size_t totalPoints = SumPoints();
    if (totalPoints > maxPoints)
    {
        for (auto cloudIt = pointsVector_.begin(); cloudIt != pointsVector_.end(); cloudIt++)
        {
            auto cloud = *cloudIt;
            if (cloud != nullptr)
            {

                int currentPointCount = cloud->GetPoints()->points_.size();
                float fractionOfTotal = (float)currentPointCount / (float)totalPoints;
                int targetPointCount = (int)(maxPoints * fractionOfTotal);
                int downsampleFactor = currentPointCount / targetPointCount;
                std::cout << "Downsampling cloud with " << 
                    currentPointCount << "points to " << 
                    targetPointCount << " points." << std::endl;
                if (downsampleFactor > 1 && targetPointCount > minPoints)
                {
                    cloud->SetPoints(cloud->GetPoints()->UniformDownSample(downsampleFactor));
                }
                if (targetPointCount < minPoints)
                {
                    cloudIt->reset();
                    pointsVector_.erase(cloudIt);
                    cloud = nullptr;
                }
            }
        }
    }
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
    Eigen::Matrix3d quatRotation = open3d::geometry::Geometry3D::GetRotationMatrixFromQuaternion(quat);
    Eigen::Matrix4d quatRotationMtx = Eigen::Matrix4d::Identity();
    quatRotationMtx.topLeftCorner(3, 3) = quatRotation;
    Eigen::Matrix4d inverseQuatRotationMtx = Eigen::Matrix4d::Identity();
    inverseQuatRotationMtx.topLeftCorner(3, 3) = quatRotation;
    inverseQuatRotationMtx.transposeInPlace();

    Eigen::Vector3d posePosition = { pose(0,3), pose(1,3), pose(2,3) };
    ImagePose posePositionMatrix = ExtractPoseTransform(pose);
    ImagePose inversePosePositionMatrix = posePositionMatrix;
    inversePosePositionMatrix(0, 3) = -posePositionMatrix(0,3);
    inversePosePositionMatrix(1, 3) = -posePositionMatrix(1,3);
    inversePosePositionMatrix(2, 3) = -posePositionMatrix(2,3);

    // Transform points by rotating 180 on Z axis and -90 on Y axis
    const Eigen::Affine3d aff = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX());
    const Eigen::Affine3d aff_cam =  aff * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mtx = posePositionMatrix * quatRotationMtx * aff.matrix();
    Eigen::Matrix4d cam_mtx = mtx * inverseQuatRotationMtx;

//    Eigen::Matrix4d cam_mtx = inversePosePositionMatrix * inverseQuatRotationMtx * aff.matrix().inverse();
//    Eigen::Matrix4d cam_mtx = inversePosePositionMatrix * inverseQuatRotationMtx * aff.matrix().inverse();
    Eigen::Matrix4d transformedPose = pose;
    std::cout << "Initial Pose: " << mtx << std::endl;
//    std::cout << "Camera Pose: " << cam_mtx << std::endl;
//    std::cout << "Transformed Pose:\n " << mtx << std::endl;

    // Do initial transformation
    points->GetPoints()->Transform(mtx);
    points->SetPose(mtx.inverse());

    // Registration
    bool doRegistration = false;
    if (doRegistration)
    {
        RegisterPoints(points);
    }

    // Add to vector
    pointsVector_.push_back(points);
    return true;
}

void Reco3D::PointsVector::RegisterPoints(std::shared_ptr<Reco3D::PointCloud>& points)
{
    if (Count() > 0)
    {
        bool transformed = false;
        int transformationCount = 0;
        Eigen::Matrix4d transformMtx = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d regMtx = Eigen::Matrix4d::Identity();
        auto& source = points;

        // Break on transformed being true
        for (auto cloudIt = pointsVector_.rbegin(); (cloudIt != pointsVector_.rend()) && (!transformed); cloudIt++)
        {
            auto& target = *cloudIt;
            double notGoodRMSE = 0.2;
            double goodEnoughRMSE = 0.025;
            double excellentRMSE = 0.01;

            auto existingRegResults = EvaluateCurrentRegistration(source, target, regMtx);
            double& currentRMSE = existingRegResults.inlier_rmse_;
            if (currentRMSE > notGoodRMSE)
            {
                std::cout << "Current point cloud is not a good match for ICP, continuing." << std::endl;
                continue;
            }
            if (currentRMSE < excellentRMSE)
            {
                std::cout << "Current RMSE " << currentRMSE << " is good enough." << std::endl;
                continue;
            }

            // Compute new registration
            auto newRegResult = RegisterPointsICP(source, target, regMtx);
            double& newRMSE = newRegResult.inlier_rmse_;
            if (newRMSE < currentRMSE && newRMSE < goodEnoughRMSE)
            {
                std::cout << "Transforming point cloud based on new reg results" << std::endl;
                source->GetPoints()->Transform(newRegResult.transformation_);
                transformMtx = newRegResult.transformation_ * transformMtx;
                transformationCount++;
                //                auto postTransformResults = EvaluateCurrentRegistration(source, target, regMtx);
                //                regMtx *= newRegResult.transformation_;
            }
            else
            {
                std::cout << "Discarding new registration results." << std::endl;
            }
            if (newRMSE < goodEnoughRMSE)
            {
                transformed = true;
            }
        }

        // Do final transformation
        source->GetPoints()->Transform(regMtx);
        std::cout << "Transformed point cloud " << transformationCount << " times." << std::endl;
        std::cout << "Final reg mtx:\n" << transformMtx << std::endl;

        if (!transformed)
        {
            std::cout << "Failed to find better ICP registration" << std::endl;
        }
    }
}

open3d::registration::RegistrationResult Reco3D::PointsVector::EvaluateCurrentRegistration(std::shared_ptr<Reco3D::PointCloud>& source, std::shared_ptr<Reco3D::PointCloud>& target, Eigen::Matrix4d& m)
{
    //        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    double max_correspondence_distance = 1000.0;

    // Evaluate current registration
    auto current_reg_result = open3d::registration::EvaluateRegistration(*source->GetPoints(), *target->GetPoints(), max_correspondence_distance, m);
    double current_rmse = current_reg_result.inlier_rmse_;
    std::cout << "Current fitness:" << current_reg_result.fitness_ << std::endl;
    std::cout << "Current RMSE:" << current_rmse << std::endl;
    std::cout << "Current transformation:" << m << std::endl;
    return current_reg_result;

//    // Transform points based on registration results
//    // ICP aligns  points(SOURCE) to pointsVector_.back()(TARGET) 
//    auto reg_result = RegisterPointsICP(source, target, m);
//    if (current_rmse > reg_result.inlier_rmse_)
//    {
//        std::cout << "New registration results are better!" << std::endl;
//    }
//    else
//    {
//        std::cout << "Using old registration results." << std::endl;
//    }
//
//    Eigen::Matrix4d reg_transform = reg_result.transformation_;
//    auto regTranslation = ExtractPoseTransform(Eigen::Matrix4d(reg_transform));
//    Eigen::Vector3d translationVector{ regTranslation(0,3),regTranslation(1,3), regTranslation(2,3) };
//    Eigen::Matrix4d regRotation = Eigen::Matrix4d::Identity();
//    regRotation.topLeftCorner(3, 3) = reg_result.transformation_.topLeftCorner(3, 3);
//
//    Eigen::Affine3d regAffine = Eigen::Affine3d::Identity();
//    regAffine *= Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
//
//    Eigen::Matrix4d finalRegMtx = reg_transform;
//
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

// Aligns SOURCE to TARGET
open3d::registration::RegistrationResult Reco3D::PointsVector::RegisterPointsICP(std::shared_ptr<Reco3D::PointCloud> source, 
    std::shared_ptr<Reco3D::PointCloud> target, Reco3D::ImagePose& target_pose)
{
     auto estimation = open3d::registration::TransformationEstimationPointToPoint(true);
     auto criteria = open3d::registration::ICPConvergenceCriteria();
     double max_correspondence_distance = 100.0;
     criteria.max_iteration_ = 60;
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

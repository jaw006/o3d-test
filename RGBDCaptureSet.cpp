#include "RGBDCaptureSet.h"

open3d::registration::RegistrationResult RegisterPoints(std::shared_ptr<Reco3D::PointCloud> source, std::shared_ptr<Reco3D::PointCloud> target)
{

         auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
         auto criteria = open3d::registration::ICPConvergenceCriteria();
         double max_correspondence_distance = 0.5;
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

std::shared_ptr<Reco3D::PointCloud> Reco3D::RGBDCaptureSet::GetSourcePointCloud()
{
    return pointsVector_->GetSourcePointCloud();
//    if (Count() == 0)
//    {
//        return nullptr;
//    }
//    else
//    {
//        return points_.at(0);
//    }
}

Reco3D::RGBDCaptureSet::RGBDCaptureSet() :
    rgbdToPoints_  (std::shared_ptr<RGBDToPoints>   (new Reco3D::RGBDToPoints())),
    pointsToMesh_  (std::shared_ptr<PointsToMesh>   (new Reco3D::PointsToMesh())),
    combinedPoints_(std::shared_ptr<o3d_PointCloud> (new Reco3D::o3d_PointCloud())),
    combinedMesh_  (std::shared_ptr<o3d_TriMesh>    (new Reco3D::o3d_TriMesh())),
    pointsVector_  (std::shared_ptr<PointsVector>   (new Reco3D::PointsVector())),
    colorMap_      (std::shared_ptr<ColorMap>       (new Reco3D::ColorMap()))
{
}

Reco3D::RGBDCaptureSet::~RGBDCaptureSet()
{
}

void Reco3D::RGBDCaptureSet::AddCapture(std::shared_ptr<RGBDCapture_t> capture)
{
    // TODO: Check duplicates?
    if (capture == nullptr)
    {
        return;
    }
    // Create point cloud object from capture
    std::shared_ptr<Reco3D::PointCloud> newPoints(rgbdToPoints_->ToPointCloud(capture));
    // Insert capture and points
    pointsVector_->AddPoints(newPoints);
    captures_.push_back(capture);

    // Clamp
    ClampMaxPointsSize();
}

// Remove all clouds
void Reco3D::RGBDCaptureSet::Clear()
{
    pointsVector_.reset(new PointsVector());
    combinedPoints_.reset(new Reco3D::o3d_PointCloud());
    combinedMesh_.reset(new Reco3D::o3d_TriMesh());
    // This might be memory unsafe
    captures_.clear();
    points_.clear();
}

size_t Reco3D::RGBDCaptureSet::Count()
{
    return captures_.size();
}



std::shared_ptr<Reco3D::PointCloud> Reco3D::RGBDCaptureSet::GetCombinedPointCloud()
{
    return pointsVector_->GetCombinedPoints();
}

const std::vector<std::shared_ptr<Reco3D::PointCloud>>& Reco3D::RGBDCaptureSet::GetPointsVector()
{
    return pointsVector_->GetPointsVector();
}

std::shared_ptr<Reco3D::o3d_TriMesh> Reco3D::RGBDCaptureSet::GetCombinedTriangleMesh()
{
    std::shared_ptr<o3d_TriMesh> mesh = pointsToMesh_->ToMesh(pointsVector_);

    // Only do color map optimization with multiple captures???
//    if (Count() > 1)
//    {
//    std::cout << "Starting Color Map Optimization" << std::endl;
//        colorMap_->ColorMapOptimization(*mesh, captures_);
//    }

    return mesh;
}

void Reco3D::RGBDCaptureSet::ClampMaxPointsSize()
{
    // To prevent output log spam
    pointsVector_->ClampMaxPointsSize();
}

size_t Reco3D::RGBDCaptureSet::SumPoints()
{
    return pointsVector_->SumPoints();
}

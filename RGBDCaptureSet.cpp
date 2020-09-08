#include "RGBDCaptureSet.h"

Reco3D::RGBDCaptureSet::RGBDCaptureSet() :
    rgbdToPoints_  (std::shared_ptr<RGBDToPoints>   (new Reco3D::RGBDToPoints())),
    pointsToMesh_  (std::shared_ptr<PointsToMesh>   (new Reco3D::PointsToMesh())),
    combinedPoints_(std::shared_ptr<o3d_PointCloud> (new Reco3D::o3d_PointCloud())),
    combinedMesh_  (std::shared_ptr<o3d_TriMesh>    (new Reco3D::o3d_TriMesh())),
    pointsVector_  (std::shared_ptr<PointsVector>   (new Reco3D::PointsVector()))
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
    // Insert capture
    captures_.push_back(*capture);
//    // Add created point cloud
    std::shared_ptr<Reco3D::PointCloud> newPoints(rgbdToPoints_->ToPointCloud(capture));
    pointsVector_->AddPoints(newPoints);
}

size_t Reco3D::RGBDCaptureSet::Count()
{
    return captures_.size();
}

std::shared_ptr<Reco3D::PointCloud> Reco3D::RGBDCaptureSet::GetCombinedPointCloud()
{
    return pointsVector_->GetCombinedPoints();
}

std::shared_ptr<Reco3D::o3d_TriMesh> Reco3D::RGBDCaptureSet::GetCombinedTriangleMesh()
{
    return pointsToMesh_->ToMesh(pointsVector_);
}

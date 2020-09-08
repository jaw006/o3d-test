#include "RGBDCaptureSet.h"

Reco3D::RGBDCaptureSet::RGBDCaptureSet() :
    rgbdToPoints_  (std::shared_ptr<RGBDToPoints>   (new Reco3D::RGBDToPoints())),
    pointsToMesh_  (std::shared_ptr<PointsToMesh>   (new Reco3D::PointsToMesh())),
    combinedPoints_(std::shared_ptr<o3d_PointCloud> (new Reco3D::o3d_PointCloud())),
    combinedMesh_  (std::shared_ptr<o3d_TriMesh>    (new Reco3D::o3d_TriMesh()))
{
}

Reco3D::RGBDCaptureSet::~RGBDCaptureSet()
{
}

void Reco3D::RGBDCaptureSet::AddCapture(std::shared_ptr<RGBDCapture_t> capture)
{
    // TODO: Check duplicates?
    if (capture != nullptr)
    {
        captures_.push_back(*capture);
    }
}

std::shared_ptr<Reco3D::o3d_PointCloud> Reco3D::RGBDCaptureSet::GetCombinedPointCloud()
{
    return std::shared_ptr<o3d_PointCloud>();
}

std::shared_ptr<Reco3D::o3d_TriMesh> Reco3D::RGBDCaptureSet::GetCombinedTriangleMesh()
{
    return std::shared_ptr<o3d_TriMesh>();
}

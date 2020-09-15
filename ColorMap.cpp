#include "ColorMap.h"

Reco3D::CameraPoseTrajectory Reco3D::ColorMap::ConstructTrajectoryFromCaptures(CaptureVector& captures)
{
    CameraPoseTrajectory camPoseTrajectory;
    std::vector<CameraPoseParameters> parameterVector;
    for (auto capture : captures)
    {
        parameterVector.push_back(capture->camPose_);
    }
    camPoseTrajectory.parameters_ = parameterVector;
    return camPoseTrajectory;
}

Reco3D::ColorMap::ImagePyramid Reco3D::ColorMap::ConstructImagePyramidFromCaptures(CaptureVector& captures)
{
    ImagePyramid pyramid;
    for (auto capture : captures)
    {
        pyramid.push_back(capture->image_);
    }
    return pyramid;
}

void Reco3D::ColorMap::ColorMapOptimization(o3d_TriMesh& mesh, CaptureVector& captures_)
{
//    open3d::geometry::RGBDImagePyramid
//    open3d::color_map::ColorMapOptimization(mesh, );
}

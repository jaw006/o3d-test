#include "ColorMap.h"


Reco3D::ColorMap::ColorMap()
{
};
Reco3D::ColorMap::~ColorMap()
{
};
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
    CameraPoseTrajectory camPoseTrajectory = ConstructTrajectoryFromCaptures(captures_);
    ImagePyramid pyramid = ConstructImagePyramidFromCaptures(captures_);
//    open3d::color_map::ColorMapOptimizationOption option();
    try
    {
        open3d::color_map::ColorMapOptimization(mesh, pyramid, camPoseTrajectory);
    }
    catch(char *e)
    {
        std::cout << e << std::endl;
    }
}

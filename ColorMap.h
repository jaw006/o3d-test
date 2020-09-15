#pragma once
#include "Types.h"
#include <Open3D/Open3D.h>
#include <Eigen/Core>

namespace Reco3D {
    // http://www.open3d.org/docs/release/cpp_api/namespaceopen3d_1_1color__map.html
class ColorMap
{
private:
    typedef std::vector<std::shared_ptr<RGBDCapture_t>> CaptureVector;
    typedef open3d::geometry::RGBDImagePyramid ImagePyramid;
    CameraPoseTrajectory ConstructTrajectoryFromCaptures(CaptureVector& captures);
    ImagePyramid ConstructImagePyramidFromCaptures(CaptureVector& captures);
public:
    ColorMap();
    ~ColorMap();
    void ColorMapOptimization(o3d_TriMesh& mesh, CaptureVector& captures_);
};
}
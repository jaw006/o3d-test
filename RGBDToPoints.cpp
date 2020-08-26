#include "RGBDToPoints.h"

Reco3D::RGBDToPoints::RGBDToPoints() :
    points_(nullptr),
    image_(nullptr),
    pose_(Eigen::Matrix4d())
{
    points_ = std::make_shared<PointCloud>();
}

Reco3D::RGBDToPoints::~RGBDToPoints()
{
}

std::shared_ptr<PointCloud> Reco3D::RGBDToPoints::ConvertToPointCloud(std::shared_ptr<RGBDImage>& image)
{
    if (image == nullptr)
    {
        return points_;
    }
    auto rgbd_image = MakeNewRGBDImage(image);
//    const RGBDImage img = *rgbd_image;
    PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);
    points_ = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_image, intrinsic);
//    points_->UniformDownSample(1);
//    points_->RemoveStatisticalOutliers(10, 0.1);
//    points_->RemoveRadiusOutliers(10, 1.0);
    return points_;
}

std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::MakeNewRGBDImage(std::shared_ptr<RGBDImage>& image)
{
    auto& color = image->color_;
    auto& depth = image->depth_;
    return RGBDImage::CreateFromColorAndDepth(color, depth, 1.0, 10000.0, false);
}

std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::SaveImage(std::shared_ptr<RGBDImage>& image)
{
    image_ = MakeNewRGBDImage(image);
    return image_;
}

void Reco3D::RGBDToPoints::SetPose(Eigen::Matrix4d& pose)
{
    pose_ = pose;
}

bool Reco3D::RGBDToPoints::ExportCapture(std::string filename)
{
    return ExportPLY(filename) && ExportPose(filename);
}

bool Reco3D::RGBDToPoints::ExportPLY(std::string filename)
{
    const std::string extension = ".ply";
    if (!points_)
        return false;
    const open3d::geometry::PointCloud& pts = *points_;
    open3d::io::WritePointCloudToPLY(DATA_DIR+filename+extension, pts, false, false, true);
    return true;
}

bool Reco3D::RGBDToPoints::ExportPose(std::string filename)
{
    const std::string extension = ".txt";
    std::string f = DATA_DIR + filename + extension;
    std::ofstream file(f);
    if (file.is_open())
    {
        file << pose_;
        file.close();
        return true;
    }
    return false;
}

bool Reco3D::RGBDToPoints::ReadPoseFromFile(std::string filename)
{
    const std::string extension = ".txt";
    std::string f = DATA_DIR + filename + extension;
    std::ifstream file(f);
    if (file.is_open())
    {
        Eigen::Matrix4d m;
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                std::string line;
                file >> std::skipws >> line;
                m(row, col) = std::stod(line);
            }
        }
        pose_ = m;
        file.close();
        return true;
    }
    return false;

}







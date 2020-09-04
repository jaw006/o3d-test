#include "RGBDToPoints.h"

Reco3D::RGBDToPoints::RGBDToPoints()
{
}

Reco3D::RGBDToPoints::~RGBDToPoints()
{
}

std::shared_ptr<Reco3D::PointCloud> Reco3D::RGBDToPoints::ToPointCloud(std::shared_ptr<RGBDCapture_t> capture)
{
    std::shared_ptr<PointCloud> output(new Reco3D::PointCloud());
    if (!capture)
    {
        return output;
    }

    const PinholeCameraIntrinsic intrinsic(1280, 720, 601.1693115234375, 600.85931396484375, 637.83624267578125, 363.8018798828125);
    auto img = open3d::geometry::PointCloud::CreateFromRGBDImage(*capture->image_, intrinsic);
    std::shared_ptr<Reco3D::PointCloud> cloud = std::make_shared<Reco3D::PointCloud>(img, capture);
    return cloud;
}
std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::MakeNewRGBDImage(std::shared_ptr<open3d::geometry::RGBDImage>& image)
{
    auto& color = image->color_;
    auto& depth = image->depth_;
    return open3d::geometry::RGBDImage::CreateFromColorAndDepth(color, depth, 1000.0, 10000.0, false);
}

//std::shared_ptr<open3d::geometry::RGBDImage> Reco3D::RGBDToPoints::SaveImage(Reco3D::ImageRGBD& image)
//{
////    image_ = Reco3D::MakeNewRGBDImage(image);
////    return image_;
//}

bool Reco3D::RGBDToPoints::ExportCapture(std::string filename, 
    std::shared_ptr<o3d_PointCloud> points, std::shared_ptr<Reco3D::RGBDCapture_t> capture)
{
    return ExportPLY(filename, points) && ExportPose(filename, capture);
}

bool Reco3D::RGBDToPoints::ExportPLY(std::string filename, std::shared_ptr<Reco3D::o3d_PointCloud> points)
{
    const std::string extension = ".ply";
    if (!points)
        return false;
    const open3d::geometry::PointCloud& pts = *points;
    open3d::io::WritePointCloudToPLY(DATA_DIR+filename+extension, pts, false, false, true);
    return true;
}

bool Reco3D::RGBDToPoints::ExportPose(std::string filename, std::shared_ptr<Reco3D::RGBDCapture_t> capture)
{
    const std::string extension = ".txt";
    std::string f = DATA_DIR + filename + extension;
    auto pose = capture->pose_;
    std::ofstream file(f);
    if (file.is_open())
    {
        file << pose;
        file.close();
        return true;
    }
    return false;
}

// Expects valid filepath with ".txt" file extension at end
Reco3D::ImagePose Reco3D::RGBDToPoints::ReadPoseFromFile(std::string filename)
{
    ImagePose pose;
    std::string f = filename;
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
        pose = m;
        file.close();
    }
    return pose;

}







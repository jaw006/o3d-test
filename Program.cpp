#include "Program.h"

using namespace open3d;

Reco3D::Program::Program(open3d::visualization::VisualizerWithKeyCallback& vis) :
    sensorConfig_(new Reco3D::IO::RGBDSensor_Config_KinectVive()),
    converter_(new Reco3D::RGBDToPoints()),
    vis_(vis)
{
    sensor_ = new Reco3D::IO::RGBDSensor_KinectVive(*sensorConfig_);
}

Reco3D::Program::~Program()
{
    delete sensorConfig_;
    sensorConfig_ = nullptr;
    delete sensor_;
    sensor_ = nullptr;
    delete converter_;
    converter_ = nullptr;
}

// sourcePath: path to file, without filename
// sourceFilename: filename, without extension
// source: points from ply are loaded into this object
bool Reco3D::LoadPlyPoseToPointCloud(std::string& sourcePath,  const std::string& sourceFilename, Reco3D::PointCloud& pointsObject) 
{
    Reco3D::RGBDToPoints converter;
    std::shared_ptr<Reco3D::o3d_PointCloud> ply_points(new Reco3D::o3d_PointCloud());
    std::string filepath = sourcePath + sourceFilename;
    if (open3d::io::ReadPointCloudFromPLY(filepath + ".ply", *ply_points, true))
    {
        pointsObject.SetPoints(ply_points);
        pointsObject.SetPose(converter.ReadPoseFromFile(filepath + ".txt"));
        std::cout << "Source exists, using that" << std::endl;
        return true;
    }
    return false;
}

void Reco3D::Program::Run()
{
    bool flag_exit = false;
    bool is_geometry_added = false;
    bool is_target_added = false;
    bool capture_source = false;
    bool capture_target = false;
    bool newSource = true;
    bool newTarget = true;
    bool clear = false;
//    visualization::VisualizerWithKeyCallback vis;
    vis_.RegisterKeyCallback(GLFW_KEY_ESCAPE,
        [&](visualization::Visualizer* vis) {
            flag_exit = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_A,
        [&](visualization::Visualizer* vis) {
            capture_source = true;
            newSource = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_T,
        [&](visualization::Visualizer* vis) {
            capture_target = true;
            newTarget = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_C,
        [&](visualization::Visualizer* vis) {
            clear = true;
            return false;
        });

    // Read existing files
//    Reco3D::RGBDToPoints source;
//    Reco3D::RGBDToPoints target;
    const std::string sourceFilename = "source";
    const std::string targetFilename = "target";
    const std::string folder = "data";
    const std::string extension = ".ply";
    const std::string delim = "/";
    std::string dataPath = folder + delim;
    std::string sourcePath = folder + delim + sourceFilename + extension;
    std::string targetPath = folder + delim + targetFilename + extension;

    Reco3D::PointCloud source;
    Reco3D::PointCloud target;
    LoadPlyPoseToPointCloud(dataPath, sourceFilename, source);
    LoadPlyPoseToPointCloud(dataPath, targetFilename, target);

    vis_.CreateVisualizerWindow("TestVisualizer", 1920, 540);
    do {
        auto im_rgbd = sensor_->CaptureFrame();
//        auto im_rgbd = sensor.CaptureFrame(enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            utility::LogInfo("Invalid capture, skipping this frame");
            continue;
        }

// -----------------------------------------------------------------
// CAPTURE SOURCE
// -----------------------------------------------------------------
        // Set source image/pose
        if (!is_geometry_added || capture_source) {
//        // TODO: Refactor this to be reusable
//            std::shared_ptr<Reco3D::o3d_PointCloud> pts;
//            if (newSource)
//            {
//                pts = source.ConvertToPointCloud(im_rgbd);
//                source.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
//                source.position_ = vtpInterface->GetTrackerPosition(selectedTrackerId);
//                source.rotation_ = vtpInterface->GetTrackerRotation(selectedTrackerId);
//                source.ExportCapture("source");
//            }
//            else
//            {
//                pts = source.points_;
//            }
//
//            source.GetPoints()->VoxelDownSample(VOXEL_SIZE);
            AddSourcePointCloud(source, vis_);

            is_geometry_added = true;
            capture_source = false;

//            if (clear)
//            {
//                vis.ClearGeometries();
//                utility::LogInfo("Clearing geometry.");
//                if (is_geometry_added)
//                {
//                    vis.AddGeometry(pts);
//                }
//                clear = false;
//                vis.UpdateRender();
//                continue;
//            }
        }


        if (!is_target_added || capture_target) {
            std::shared_ptr<PointCloud> pts2;
//            if (newTarget)
//            {
//                pts2 = target.ConvertToPointCloud(im_rgbd);
//                target.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
//                target.position_ = vtpInterface->GetTrackerPosition(selectedTrackerId);
//                target.rotation_ = vtpInterface->GetTrackerRotation(selectedTrackerId);
//                target.ExportCapture("target");
//            }
//            else
//            {
//                pts2 = target.points_;
//            }
//            target.points_->VoxelDownSample(VOXEL_SIZE);

            utility::LogInfo("Updating target.");
            target.SetPose(Reco3D::ImagePose(source.GetPose().inverse() * target.GetPose())); // Put target relative to source
//            pts2->Transform(source.pose_.inverse()*target.pose_); // This works I'm pretty sure


    // -----------------------------------------------------------------
    // REGISTRATION 
    // -----------------------------------------------------------------
             auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
             auto criteria = open3d::registration::ICPConvergenceCriteria();
             double max_correspondence_distance = 0.5;
             criteria.max_iteration_ = 30;
//             auto reg_result = registration::EvaluateRegistration(*source.points_, *target.points_, 1.0, target.pose_);
             auto reg_result = registration::RegistrationICP(
                 *source.GetPoints(),
                 *target.GetPoints(),
                 max_correspondence_distance,
                 target.GetPose(),
                 estimation,
                 criteria
             );
////            // Print fitness, RMSE
            double& fitness = reg_result.fitness_; 
            double& rmse = reg_result.inlier_rmse_;
            std::string log1 = "Fitness= " + std::to_string(fitness) + "\n";
            std::string log2 = "RMSE= " + std::to_string(rmse) + "\n";
            std::cout << "Transformation Estimation:\n" << reg_result.transformation_ << std::endl;
            utility::LogInfo(log1.c_str());
            utility::LogInfo(log2.c_str());

            target.GetPoints()->Transform(reg_result.transformation_);
//                vis.RemoveGeometry(target.points_);
            capture_target = false;

// -----------------------------------------------------------------
// ADD ALIGNED POINT CLOUDS 
// -----------------------------------------------------------------

            vis_.AddGeometry(target.GetPoints());
            is_target_added = true;
        }

// -----------------------------------------------------------------
// RENDER
// -----------------------------------------------------------------

        vis_.UpdateGeometry();
        vis_.PollEvents();
        vis_.UpdateRender();
    } while (!flag_exit);



  // -----------------------------------------------------------------
  // CLEANUP
  // -----------------------------------------------------------------
}

void Reco3D::Program::AddSourcePointCloud(Reco3D::PointCloud& source, open3d::visualization::VisualizerWithKeyCallback& vis)
{
    source.GetPoints()->Transform(source.GetPose().inverse());

    std::cout << "SourcePose:" << source.GetPose() << std::endl;

    utility::LogInfo("Updating geo.");

    vis.AddGeometry(source.GetPoints());
}

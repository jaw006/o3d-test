#include "Program.h"

using namespace open3d;

Reco3D::Program::Program(open3d::visualization::VisualizerWithKeyCallback& vis) :
    sensorConfig_(new Reco3D::IO::RGBDSensor_Config_KinectVive()),
    converter_(new Reco3D::RGBDToPoints()),
    vis_(vis)
{
    // TODO: Implement live capture again
    // sensor_ = new Reco3D::IO::RGBDSensor_KinectVive(*sensorConfig_);
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
    // Status booleans
    bool flag_exit = false;
    bool is_geometry_added = false;
    bool is_target_added = false;
    bool capture_source = false;
    bool capture_target = false;
    bool newSource = true;
    bool newTarget = true;
    bool clear = false;
    bool update_render = false;

    // Input key callbacks
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
    vis_.RegisterKeyCallback(GLFW_KEY_W,
        [&](visualization::Visualizer* vis) {
            vis->GetRenderOption().ToggleMeshShowWireframe();
            return false;
        });

    // Show backfaces
    vis_.GetRenderOption().ToggleMeshShowBackFace();
    vis_.GetRenderOption().ToggleMeshShowWireframe();

    const std::string sourceFilename = "source";
    const std::string targetFilename = "target";
    const std::string folder = "data";
    const std::string extension = ".ply";
    const std::string delim = "/";
    std::string dataPath = folder + delim;
    std::string sourcePath = folder + delim + sourceFilename + extension;
    std::string targetPath = folder + delim + targetFilename + extension;

    // Read existing files
    Reco3D::PointCloud source;
    Reco3D::PointCloud target;
    LoadPlyPoseToPointCloud(dataPath, sourceFilename, source);
    LoadPlyPoseToPointCloud(dataPath, targetFilename, target);

    vis_.CreateVisualizerWindow("TestVisualizer", 1920, 540);
    do {
        // Doing this every frame is very taxing on CPU
//        auto im_rgbd = sensor_->CaptureFrame();
//        if (im_rgbd == nullptr) {
//            utility::LogInfo("Invalid capture, skipping this frame");
//            continue;
//        }

// -----------------------------------------------------------------
// CAPTURE SOURCE
// -----------------------------------------------------------------
        // Set source image/pose
        if (!is_geometry_added || capture_source) {
            AddSourcePointCloud(source, vis_);

            // Flag for update
            is_geometry_added = true;
            update_render = true;
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
        } // End Source


        if (!is_target_added || capture_target) {
//            target.points_->VoxelDownSample(VOXEL_SIZE);
            utility::LogInfo("Updating target.");
            target.SetPose(Reco3D::ImagePose(source.GetPose().inverse() * target.GetPose())); // Put target relative to source
            // -----------------------------------------------------------------
            // REGISTRATION  (TODO: Refactor registration)
            // -----------------------------------------------------------------

             auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
             auto criteria = open3d::registration::ICPConvergenceCriteria();
             double max_correspondence_distance = 0.5;
             criteria.max_iteration_ = 30;
             auto reg_result = registration::RegistrationICP(
                 *source.GetPoints(),
                 *target.GetPoints(),
                 max_correspondence_distance,
                 target.GetPose(),
                 estimation,
                 criteria
             );
            // Print fitness, RMSE
            double& fitness = reg_result.fitness_; 
            double& rmse = reg_result.inlier_rmse_;
            std::string log1 = "Fitness= " + std::to_string(fitness) + "\n";
            std::string log2 = "RMSE= " + std::to_string(rmse) + "\n";
            std::cout << "Transformation Estimation:\n" << reg_result.transformation_ << std::endl;
            utility::LogInfo(log1.c_str());
            utility::LogInfo(log2.c_str());
            target.GetPoints()->Transform(reg_result.transformation_);

            // -----------------------------------------------------------------
            // ADD ALIGNED POINT CLOUDS 
            // -----------------------------------------------------------------
//            vis_.AddGeometry(target.GetPoints());


            // -----------------------------------------------------------------
            // MESH THE COMBINED POINTS
            // -----------------------------------------------------------------
            PointsToMesh ptsToMesh_;
            PointsVector ptsVector_;
            ptsVector_.AddPoints(std::make_shared<Reco3D::PointCloud>(source));
            ptsVector_.AddPoints(std::make_shared<Reco3D::PointCloud>(target));
            auto mesh = ptsToMesh_.ToMesh(std::make_shared<Reco3D::PointsVector>(ptsVector_));
            vis_.AddGeometry(mesh);

            // Flag for update
            capture_target = false;
            update_render = true;
            is_target_added = true;
        } // End target





// -----------------------------------------------------------------
// RENDER
// -----------------------------------------------------------------
        if (update_render)
        {
            vis_.UpdateGeometry();
            update_render = false;
        }

        vis_.PollEvents();
        vis_.UpdateRender();
    } while (!flag_exit);

  // -----------------------------------------------------------------
  // EXIT CLEANUP
  // -----------------------------------------------------------------
}

// TODO Refactor this
void Reco3D::Program::AddSourcePointCloud(Reco3D::PointCloud& source, open3d::visualization::VisualizerWithKeyCallback& vis)
{
    source.GetPoints()->Transform(source.GetPose().inverse());

    std::cout << "SourcePose:" << source.GetPose() << std::endl;

    utility::LogInfo("Updating geo.");

//    vis.AddGeometry(source.GetPoints());
}

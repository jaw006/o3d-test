#include "Program.h"

using namespace open3d;

Reco3D::Program::Program(open3d::visualization::VisualizerWithKeyCallback& vis) :
    sensorConfig_(new Reco3D::IO::RGBDSensor_Config_KinectVive()),
    converter_(new Reco3D::RGBDToPoints()),
    captureSet_(new Reco3D::RGBDCaptureSet()),
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
    delete captureSet_;
    captureSet_ = nullptr;
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

void Reco3D::Program::ExportAllPoints()
{
    std::time_t result = std::time(nullptr);
    std::string file_prefix = std::to_string(result) + "_";
    uint64_t captureNum = 0;
    auto pointsVector = captureSet_->GetPointsVector();
    for (auto pointIt = pointsVector.begin(); pointIt != pointsVector.end(); pointIt++)
    {
        std::string filename = file_prefix + std::to_string(captureNum);
        auto capture = (*pointIt)->GetCapture();
        auto points = (*pointIt)->GetPoints();
        converter_->ExportCapture(filename, points, capture);
        captureNum++;
    }
}

void SetGeometryPose(open3d::geometry::Geometry3D& object, Eigen::Matrix4d& pose, Eigen::Matrix4d& newPose)
{
    object.Transform(pose.inverse());
    pose = newPose;
    object.Transform(newPose);
}

void Reco3D::Program::AddTrackerOriginMeshes(const std::shared_ptr<open3d::geometry::TriangleMesh>& trackerMesh, const std::shared_ptr<open3d::geometry::TriangleMesh>& origin)
{
    vis_.AddGeometry(trackerMesh);
    vis_.AddGeometry(origin);
    vis_.UpdateGeometry();
    vis_.UpdateRender();
}

void Reco3D::Program::Run()
{
    // Verbosity level
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    // Status booleans
    bool flag_exit = false;
    bool is_geometry_added = false;
    bool capture_frame = false;      // Should be false on startup
    bool make_triangle_mesh = false; // Whether to turn cloud into mesh
    bool clear = false;
    bool update_render = false;
    bool show_tracker = true;
    bool update_camera = false;

    // TRACKER VISUALIZATION 
    Eigen::Matrix4d trackerPose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d originPose = Eigen::Matrix4d::Identity();
    std::shared_ptr<o3d_TriMesh> origin = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    std::shared_ptr<o3d_TriMesh> trackerMesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame();

    // Input key callbacks
    vis_.RegisterKeyCallback(GLFW_KEY_ESCAPE,
        [&](visualization::Visualizer* vis) {
            flag_exit = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_A,
        [&](visualization::Visualizer* vis) {
            capture_frame = !capture_frame;
//            newSource = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_M,
        [&](visualization::Visualizer* vis) {
            make_triangle_mesh = !make_triangle_mesh;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_E,
        [&](visualization::Visualizer* vis) {
            ExportAllPoints();
//            newSource = true;
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_C,
        [&](visualization::Visualizer* vis) {
//            clear = true;
            captureSet_->Clear();
            vis->ClearGeometries();
            AddTrackerOriginMeshes(trackerMesh, origin);
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_W,
        [&](visualization::Visualizer* vis) {
            vis->GetRenderOption().ToggleMeshShowWireframe();
            return false;
        });
    vis_.RegisterKeyCallback(GLFW_KEY_T,
        [&](visualization::Visualizer* vis) {
            show_tracker = !show_tracker;
            return false;
        });

    // Show backfaces
    vis_.GetRenderOption().ToggleMeshShowBackFace();
//    vis_.GetRenderOption().ToggleMeshShowWireframe();

    const std::string sourceFilename = "source";
    const std::string targetFilename = "target";
    const std::string folder = "data";
    const std::string extension = ".ply";
    const std::string delim = "/";
    std::string dataPath = folder + delim;
    std::string sourcePath = folder + delim + sourceFilename + extension;
    std::string targetPath = folder + delim + targetFilename + extension;

//    // Read existing files
//    Reco3D::PointCloud source;
//    Reco3D::PointCloud target;
//    LoadPlyPoseToPointCloud(dataPath, sourceFilename, source);
//    LoadPlyPoseToPointCloud(dataPath, targetFilename, target);
//    open3d::visualization::glsl::CoordinateFrameRenderer coords;

    vis_.CreateVisualizerWindow("TestVisualizer", 1920, 540);

    vis_.AddGeometry(trackerMesh);
    vis_.AddGeometry(origin);

    do {
// -----------------------------------------------------------------
// New simpler main loop 
// -----------------------------------------------------------------
        if (capture_frame)
        {
            if (!sensor_)
            {
                return;
            }
            std::cout << "Capturing frame!" << std::endl;
            auto im_rgbd = sensor_->CaptureFrame();
            if (im_rgbd == nullptr) {
                utility::LogInfo("Invalid capture, skipping this frame");
                continue;
            }
            else {
//                capture_frame = false;
                update_render = true;
                update_camera = true;
                captureSet_->AddCapture(im_rgbd);

                // Add point cloud or triangle mesh
                if(!make_triangle_mesh)
                {
                    std::cout << "Adding point cloud!" << std::endl;
                    auto pointsVector = captureSet_->GetPointsVector();
                    // Add last created point cloud
                    auto lastAddedPoints = (pointsVector.back());
                    vis_.AddGeometry(lastAddedPoints->GetPoints());
                    // Set origin to camera pose
                    SetGeometryPose(*origin, originPose, lastAddedPoints->GetPose());
                }
                else
                {
                    {
                        std::cout << "Adding triangle mesh!" << std::endl;
                        vis_.ClearGeometries();
                        AddTrackerOriginMeshes(trackerMesh, origin);
                        vis_.AddGeometry(captureSet_->GetCombinedTriangleMesh());
                    }
                }
                is_geometry_added = true;
                std::cout << "Done!" << std::endl;

            }
        }

        // TRACKER POSE
//        Eigen::Matrix4d trackerPose = sensor_->GetTrackerPose();
//        std::cout << trackerPose << std::endl;
// -----------------------------------------------------------------
// RENDER
// -----------------------------------------------------------------
        if (update_render)
        {
            std::cout << "Updating geometry!" << std::endl;
            if (is_geometry_added)
            {
//                vis_.UpdateGeometry(captureSet_->GetCombinedTriangleMesh());
                vis_.UpdateGeometry();
            }
            update_render = false;
        }
        // Update tracker
        if (show_tracker)
        {
            SetGeometryPose(*trackerMesh, trackerPose, sensor_->GetTrackerPose());
        }
        
        vis_.UpdateGeometry();
        vis_.PollEvents();

        // Update camera position if captured frame
        if (update_camera)
        {
            vis_.GetViewControl().GetModelMatrix();
        }
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

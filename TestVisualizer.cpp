// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <k4a/k4a.h>

#include <math.h>
#include <atomic>
#include <csignal>
#include <ctime>

#include <iostream>
#include <memory>
#include <thread>

#include <Open3D/Open3D.h>

#include "RGBDToPoints.h"
#include "VTPLibInterface.h"
#include "Open3D/Visualization/Utility/GLHelper.h"


using namespace open3d;
using namespace Reco3D;

void PrintUsage() {
    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Options: ");
    utility::LogInfo("--config  Config .json file (default: none)");
    utility::LogInfo("--list    List the currently connected K4A devices");
    utility::LogInfo("--device  Specify the device index to use (default: 0)");
    utility::LogInfo("-a        Align depth with color image (default: disabled)");
    utility::LogInfo("-h        Print this helper");
    // clang-format on
}

int main(int argc, char** argv) {
    // Taken verbatim from Open3D examples "AzureKinectViewer"
    // Parse arguments
    if (utility::ProgramOptionExists(argc, argv, "-h")) {
        PrintUsage();
        return 0;
    }

    if (utility::ProgramOptionExists(argc, argv, "--list")) {
        io::AzureKinectSensor::ListDevices();
        return 0;
    }
    
    io::AzureKinectSensorConfig sensor_config;
    if (utility::ProgramOptionExists(argc, argv, "--config")) {
        auto config_filename =
            utility::GetProgramOptionAsString(argc, argv, "--config", "");
        if (!io::ReadIJsonConvertibleFromJSON(config_filename, sensor_config)) {
            utility::LogInfo("Invalid sensor config");
            return 1;
        }
    }
    else {
        utility::LogInfo("Use default sensor config");
    }

    int sensor_index =
        utility::GetProgramOptionAsInt(argc, argv, "--device", 0);
    if (sensor_index < 0 || sensor_index > 255) {
        utility::LogWarning("Sensor index must between [0, 255]: {}",
            sensor_index);
        return 1;
    }

//    bool enable_align_depth_to_color =
//        utility::ProgramOptionExists(argc, argv, "-a");
    bool enable_align_depth_to_color = true;
        
    // -----------------------------------------------------------------
    // VTPLIB
    // -----------------------------------------------------------------

    TrackerId preferredTrackerId = 3;
    TrackerId selectedTrackerId = 0;
    std::unique_ptr<VTPLibInterface> vtpInterface(new VTPLibInterface());
    auto ids = vtpInterface->GetTrackerIds();

    if (ids.empty())
    {
        std::cout << "Warning: No VIVE Trackers found!";
    }
    else
    {
        std::cout << "Found devices with ids:(";
        for(auto id : ids)
        {
            std::cout << id << " ";
        }
        std::cout << std::endl;
        // If preferredTrackerId returns identity matrix, use the first other tracker found for position
        if (!vtpInterface->GetTrackerMatrix4d(preferredTrackerId).isIdentity())
        {
            selectedTrackerId = preferredTrackerId;
        }
        else
        {
            selectedTrackerId = ids[0];
        }
    }
    std::cout << "Using tracker id# " << selectedTrackerId << "for pose" << std::endl;

    vtpInterface->testVTPLibInterface();
    Reco3D::RGBDToPoints test;
    test.ReadPoseFromFile("testpose");
    std::cout << "Printing test pose:\n" << test.pose_ << std::endl;


    // -----------------------------------------------------------------
    // BEGIN ALGORITHM
    // -----------------------------------------------------------------
    // Init RGBDToPoints


    // Init sensor
    io::AzureKinectSensor sensor(sensor_config);
    if (!sensor.Connect(sensor_index)) {
        utility::LogWarning("Failed to connect to sensor, abort.");
        return 1;
    }

    // Start viewing
    bool flag_exit = false;
    bool is_geometry_added = false;
    bool is_target_added = false;
    bool capture_source = false;
    bool capture_target = false;
    bool newSource = true;
    bool newTarget = true;
    bool clear = false;
    visualization::VisualizerWithKeyCallback vis;
    vis.RegisterKeyCallback(GLFW_KEY_ESCAPE,
        [&](visualization::Visualizer* vis) {
            flag_exit = true;
            return false;
        });
    vis.RegisterKeyCallback(GLFW_KEY_A,
        [&](visualization::Visualizer* vis) {
            capture_source = true;
            newSource = true;
            return false;
        });
    vis.RegisterKeyCallback(GLFW_KEY_T,
        [&](visualization::Visualizer* vis) {
            capture_target = true;
            newTarget = true;
            return false;
        });
    vis.RegisterKeyCallback(GLFW_KEY_C,
        [&](visualization::Visualizer* vis) {
            clear = true;
            return false;
        });

    // Restore view
    //const std::string window_name = "";
    //vis.CreateVisualizerWindow(window_name);

    Reco3D::RGBDToPoints source;
    Reco3D::RGBDToPoints target;
    open3d::geometry::PointCloud existingSource;
    open3d::geometry::PointCloud existingTarget;
    // Read existing files
    if (open3d::io::ReadPointCloudFromPLY("data/source.ply", existingSource, true))
    {
        newSource = false;
        source.points_ = std::make_shared<PointCloud>(existingSource);
        source.ReadPoseFromFile("source");
        std::cout << "Source exists, using that" << std::endl;
    }
    if (open3d::io::ReadPointCloudFromPLY("data/target.ply", existingTarget, true))
    {
        newTarget = false;
        target.points_ = std::make_shared<PointCloud>(existingTarget);
        target.ReadPoseFromFile("target");
        std::cout << "Target exists, using that" << std::endl;
    }

    vis.CreateVisualizerWindow("A", 1920, 540);
    do {
        auto im_rgbd = sensor.CaptureFrame(enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            utility::LogInfo("Invalid capture, skipping this frame");
            continue;
        }
// -----------------------------------------------------------------
// CAPTURE SOURCE
// -----------------------------------------------------------------

        // Set source image/pose
        if (!is_geometry_added || capture_source) {
            std::shared_ptr<PointCloud> pts;
            if (newSource)
            {
                pts = source.ConvertToPointCloud(im_rgbd);
                source.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
                source.position_ = vtpInterface->GetTrackerPosition(selectedTrackerId);
                source.rotation_ = vtpInterface->GetTrackerRotation(selectedTrackerId);
                source.ExportCapture("source");
            }
            else
            {
                pts = source.points_;
            }

            pts->Transform(source.pose_.inverse());

            std::cout << "SourcePose:" << source.pose_ << std::endl;

            utility::LogInfo("Updating geo.");

            vis.AddGeometry(pts);

            is_geometry_added = true;
            capture_source = false;

            if (clear)
            {
                vis.ClearGeometries();
                utility::LogInfo("Clearing geometry.");
                if (is_geometry_added)
                {
                    vis.AddGeometry(pts);
                }
                clear = false;
                vis.UpdateRender();
                continue;
            }
        }


        if (!is_target_added || capture_target) {
            std::shared_ptr<PointCloud> pts2;
            if (newTarget)
            {
                pts2 = target.ConvertToPointCloud(im_rgbd);
                target.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
                target.position_ = vtpInterface->GetTrackerPosition(selectedTrackerId);
                target.rotation_ = vtpInterface->GetTrackerRotation(selectedTrackerId);
                target.ExportCapture("target");
            }
            else
            {
                pts2 = target.points_;
            }

            utility::LogInfo("Updating target.");

            pts2->Transform(source.pose_.inverse()*target.pose_);
            pts2->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
            vis.AddGeometry(target.points_);
            is_target_added = true;

    // -----------------------------------------------------------------
    // REGISTRATION 
    // -----------------------------------------------------------------

// 
//             // Registration step
//             auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
//             auto criteria = open3d::registration::ICPConvergenceCriteria();
//             // Transformation between source and target is the difference between the two matrices
//             auto diff_pos = (source.position_ - target.position_);
//             Eigen::Matrix4d diff_pose = target.pose_ - source.pose_;
// 
// 
// 
// //            std::cout << "Diffpose: " << diff_pose << std::endl;
// //            criteria.max_iteration_ = 30;
// //            auto reg_result = registration::EvaluateRegistration(*source.points_, *target.points_, 1.0, diff_pose);
// ////            // Print fitness, RMSE
// //            double& fitness = reg_result.fitness_; 
// //            double& rmse = reg_result.inlier_rmse_;
// //            std::string log1 = "Fitness= " + std::to_string(fitness) + "\n";
// //            std::string log2 = "RMSE= " + std::to_string(rmse) + "\n";
// //            std::cout << "Transformation Estimation:\n" << reg_result.transformation_ << std::endl;
// //            utility::LogInfo(log1.c_str());
// //            utility::LogInfo(log2.c_str());
// //                vis.RemoveGeometry(target.points_);
//                capture_target = false;
        }
        vis.UpdateGeometry();
        vis.PollEvents();
        vis.UpdateRender();
    } while (!flag_exit);



    // -----------------------------------------------------------------
    // CLEANUP
    // -----------------------------------------------------------------

    return 0;
}

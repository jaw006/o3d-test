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


    // -----------------------------------------------------------------
    // BEGIN ALGORITHM
    // -----------------------------------------------------------------
    // Init RGBDToPoints
    Reco3D::RGBDToPoints source;
    Reco3D::RGBDToPoints target;


    // Init sensor
    io::AzureKinectSensor sensor(sensor_config);
    if (!sensor.Connect(sensor_index)) {
        utility::LogWarning("Failed to connect to sensor, abort.");
        return 1;
    }

    // Start viewing
    bool flag_exit = false;
    bool is_geometry_added = false;
    bool capture_source = false;
    bool capture_target = false;
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
            return false;
        });
    vis.RegisterKeyCallback(GLFW_KEY_T,
        [&](visualization::Visualizer* vis) {
            capture_target = true;
            return false;
        });
    vis.RegisterKeyCallback(GLFW_KEY_C,
        [&](visualization::Visualizer* vis) {
            clear = true;
            return false;
        });

    //const std::string window_name = "";
    //vis.CreateVisualizerWindow(window_name);

    vis.CreateVisualizerWindow("A", 1920, 540);
    do {
        auto im_rgbd = sensor.CaptureFrame(enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            utility::LogInfo("Invalid capture, skipping this frame");
            continue;
        }

        // Set source image/pose
        if (!is_geometry_added || capture_source) {
            auto pts = source.ConvertToPointCloud(im_rgbd);
            source.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
            std::cout << "SourcePose:" << source.pose_ << std::endl;
            utility::LogInfo("Updating geo.");
            vis.AddGeometry(pts);
//            source.SaveImage(im_rgbd);
            is_geometry_added = true;
            capture_source = false;
//            source.points_->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
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
    // -----------------------------------------------------------------
    // REGISTRATION 
    // -----------------------------------------------------------------

        if (capture_target) {
            auto pts2 = target.ConvertToPointCloud(im_rgbd);
            target.SetPose(vtpInterface->GetTrackerMatrix4d(selectedTrackerId));
            std::cout << "TargetPose:" << target.pose_ << std::endl;

            utility::LogInfo("Updating target.");
            // Registration step
            auto estimation = open3d::registration::TransformationEstimationPointToPoint(false);
            auto criteria = open3d::registration::ICPConvergenceCriteria();
            // Transformation between source and target is the difference between the two matrices
            auto init = target.pose_ - source.pose_;
            criteria.max_iteration_ = 30;
//            target.SaveImage(im_rgbd);
            
            std::cout << "Target-Source:" << init << std::endl;

            auto registration = registration::RegistrationICP(*source.points_, *pts2, 1.0, init,
                    estimation, criteria);
            pts2->Transform(registration.transformation_);

            // Print fitness, RMSE
            double& fitness = registration.fitness_; 
            double& rmse = registration.inlier_rmse_;
            std::string log = "Fitness= " + std::to_string(fitness) + ";RMSE= " + std::to_string(rmse);

//            target.points_->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
            utility::LogInfo(log.c_str());
            vis.RemoveGeometry(target.points_);
            vis.AddGeometry(pts2);
            vis.GetViewControl().FitInGeometry(*pts2);

//            is_geometry_added = true;
            capture_target = false;
        }
//        if (capture_image)
//        {
//            vis.UpdateGeometry();
//            capture_image = false;
//        }
        vis.UpdateGeometry();
        vis.PollEvents();
        vis.UpdateRender();

        // Update visualizer

    } while (!flag_exit);



    // -----------------------------------------------------------------
    // CLEANUP
    // -----------------------------------------------------------------

    return 0;
}

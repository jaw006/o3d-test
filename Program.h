#pragma once
#include <math.h>
#include <atomic>
#include <csignal>
#include <ctime>
#include <memory>
#include <iostream>
#include <thread>

#include "RGBD_IO.h"
#include "RGBDToPoints.h"
#include "PointCloud.h"

#include <Open3D/Open3D.h>

#include "RGBDToPoints.h"
#include "VTPLibInterface.h"
#include "Open3D/Visualization/Utility/GLHelper.h"
#include "Types.h"
#include "Program.h"


namespace Reco3D
{
    class Program
    {
    private:
        Reco3D::IO::RGBDSensor_KinectVive*        sensor_;
        Reco3D::IO::RGBDSensor_Config_KinectVive* sensorConfig_;
        Reco3D::RGBDToPoints*                     converter_;
        open3d::visualization::VisualizerWithKeyCallback& vis_;

//        void LoadPlyToPointCloud(std::string& sourcePath, Reco3D::o3d_PointCloud& existingSource, Reco3D::PointCloud& source, const std::string& sourceFilename);

    public:
        Program(open3d::visualization::VisualizerWithKeyCallback& vis);
        ~Program();
        void Run();
        void AddSourcePointCloud(Reco3D::PointCloud& source, open3d::visualization::VisualizerWithKeyCallback& vis);
    };


    bool LoadPlyPoseToPointCloud(std::string& sourcePath, const std::string& sourceFilename, Reco3D::PointCloud& pointsObject);

}
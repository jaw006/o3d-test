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
#include "PointsToMesh.h"
#include "RGBDCaptureSet.h"


namespace Reco3D
{
    class Program
    {
    private:
        // Why does RGBDCaptureSet have a PointsVector? 
        Reco3D::IO::RGBDSensor_KinectVive*        sensor_;
        Reco3D::IO::RGBDSensor_Config_KinectVive* sensorConfig_;
        Reco3D::RGBDToPoints*                     converter_;
        Reco3D::RGBDCaptureSet*                   captureSet_;
        open3d::visualization::VisualizerWithKeyCallback& vis_;

    public:
        Program(open3d::visualization::VisualizerWithKeyCallback& vis);
        ~Program();
        void Run();

        // Helper methods
        // This should probably go in a class that manages the point cloud scene
        void AddSourcePointCloud(Reco3D::PointCloud& source, open3d::visualization::VisualizerWithKeyCallback& vis);

    };

    // TODO: Make class member
    bool LoadPlyPoseToPointCloud(std::string& sourcePath, const std::string& sourceFilename, Reco3D::PointCloud& pointsObject);

}
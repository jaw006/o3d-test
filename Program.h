#pragma once
#include <memory>
#include <iostream>
#include "RGBD_IO.h"
#include "RGBDToPoints.h"
#include "PointCloud.h"

namespace Reco3D
{
    class Program
    {
    private:
        Reco3D::IO::RGBDSensor_KinectVive*        sensor_;
        Reco3D::IO::RGBDSensor_Config_KinectVive* sensorConfig_;
        Reco3D::RGBDToPoints*                     converter_;
        open3d::visualization::VisualizerWithKeyCallback* vis_;

    public:
        Program(open3d::visualization::VisualizerWithKeyCallback* vis);
        ~Program();
        void Run();
    };

}
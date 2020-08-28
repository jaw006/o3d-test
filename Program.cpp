#include "Program.h"

Reco3D::Program::Program(open3d::visualization::VisualizerWithKeyCallback* vis) :
    sensorConfig_(new Reco3D::IO::RGBDSensor_Config_KinectVive()),
    sensor_(new Reco3D::IO::RGBDSensor_KinectVive(*sensorConfig_)),
    converter_(new Reco3D::RGBDToPoints()),
    vis_(vis)
{
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

void Reco3D::Program::Run()
{
    do
    {

    }
    while(true);
}

#include "RGBD_IO.h"

Reco3D::IO::RGBDSensor_KinectVive::RGBDSensor_KinectVive(RGBDSensor_Config_KinectVive config) :
    sensor_(nullptr),
    vtplib_(nullptr),
    config_(config)

{
    InitializeAzureKinect();
    InitializeVTPLib();
}

Reco3D::IO::RGBDSensor_KinectVive::~RGBDSensor_KinectVive()
{
}

std::shared_ptr<Reco3D::RGBDCapture_t> Reco3D::IO::RGBDSensor_KinectVive::CaptureFrame()
{
    return std::shared_ptr<RGBDCapture_t>();
}

bool Reco3D::IO::RGBDSensor_KinectVive::InitializeAzureKinect()
{
    return false;
}

bool Reco3D::IO::RGBDSensor_KinectVive::InitializeVTPLib()
{
    return false;
}

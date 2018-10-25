#include "Controller.hpp"

#include <iostream>

Controller::~Controller()
{
}

ControllerType Controller::getType()
{
    return type;
}

void Controller::statCallback(ConstWorldStatisticsPtr &msg)
{
    (void)msg;
    //std::cout << "Un-overwritten statCallback called" << std::endl;
}

void Controller::poseCallback(ConstPosesStampedPtr &msg)
{
    (void)msg;
    //std::cout << "Un-overwritten poseCallback called" << std::endl;
}

void Controller::cameraCallback(ConstImageStampedPtr &msg)
{
    (void)msg;
    //std::cout << "Un-overwritten cameraCallback called" << std::endl;
}

void Controller::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    (void)msg;
    //std::cout << "Un-overwritten lidarCallback called" << std::endl;
}

ControlOutput Controller::getControlOutput()
{
    return ControlOutput{0, 0};
}

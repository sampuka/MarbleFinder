#include "Controller.hpp"

#include <iostream>

Controller::~Controller()
{
}

void Controller::statCallback()
{
    std::cout << "Un-overwritten statCallback called" << std::endl;
}

void Controller::poseCallback()
{
    std::cout << "Un-overwritten poseCallback called" << std::endl;
}

void Controller::cameraCallback()
{
    std::cout << "Un-overwritten cameraCallback called" << std::endl;
}

void Controller::lidarCallback()
{
    std::cout << "Un-overwritten lidarCallback called" << std::endl;
}

ControllerType Controller::getType()
{
    return type;
}

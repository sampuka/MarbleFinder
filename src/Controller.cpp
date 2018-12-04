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
    int pose_size = msg->pose_size();

    for (int i = 0; i < pose_size; i++)
    {
        if (msg->pose(i).name() == "pioneer2dx")
        {
            posf.x = msg->pose(i).position().x();
            posf.y = msg->pose(i).position().y();

            double w = msg->pose(i).orientation().w();
            double x = msg->pose(i).orientation().x();
            double y = msg->pose(i).orientation().y();
            double z = msg->pose(i).orientation().z();

            double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
            double pitch;
            double sinp = 2.0 * (w * y - z * x);
            if (std::fabs(sinp) >= 1)
                pitch = std::copysign(M_PI / 2, sinp);
            else
                pitch = std::asin(sinp);
            dir = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            stable = roll < 0.001 && pitch < 0.001;

            // Magic from Wikipedia
            // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        }
    }
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

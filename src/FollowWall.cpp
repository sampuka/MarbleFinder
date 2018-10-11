#include "FollowWall.hpp"

#include <iostream>

FollowWall::FollowWall()
{
    type = ControllerType::FollowWall;
}

/*
void FollowWall::statCallback(ConstWorldStatisticsPtr &msg)
{
    std::cout << "Un-overwritten statCallback called" << std::endl;
}
*/

/*
void FollowWall::poseCallback(ConstPosesStampedPtr &msg)
{
    std::cout << "Un-overwritten poseCallback called" << std::endl;
}
*/

void FollowWall::cameraCallback(ConstImageStampedPtr &msg)
{
    (void)msg;
    std::cout << "Empty FollowWall cameraCallback called" << std::endl;
}

void FollowWall::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    (void)msg;
    std::cout << "Empty FollowWall lidarCallback called" << std::endl;
}

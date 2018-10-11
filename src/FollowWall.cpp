#include "FollowWall.hpp"

#include <iostream>

FollowWall::FollowWall()
{
    type = ControllerType::FollowWall;
}

/*
void FollowWall::statCallback()
{
    std::cout << "Un-overwritten statCallback called" << std::endl;
}
*/

/*
void FollowWall::poseCallback()
{
    std::cout << "Un-overwritten poseCallback called" << std::endl;
}
*/

void FollowWall::cameraCallback()
{
    std::cout << "Un-overwritten cameraCallback called" << std::endl;
}

void FollowWall::lidarCallback()
{
    std::cout << "Un-overwritten lidarCallback called" << std::endl;
}

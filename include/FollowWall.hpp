#ifndef FOLLOWWALL_HPP
#define FOLLOWWALL_HPP

#include "Controller.hpp"

class FollowWall : public Controller
{
public:
    FollowWall();

    //virtual void statCallback() override;
    //virtual void poseCallback() override;
    virtual void cameraCallback() override;
    virtual void lidarCallback() override;
};

#endif // FOLLOWWALL_HPP

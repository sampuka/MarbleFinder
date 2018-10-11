#ifndef FOLLOWWALL_HPP
#define FOLLOWWALL_HPP

#include "Controller.hpp"

class FollowWall : public Controller
{
public:
    FollowWall();

    //virtual void statCallback(ConstWorldStatisticsPtr &msg) override;
    //virtual void poseCallback(ConstPosesStampedPtr &msg) override;
    virtual void cameraCallback(ConstImageStampedPtr &msg) override;
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg) override;
};

#endif // FOLLOWWALL_HPP

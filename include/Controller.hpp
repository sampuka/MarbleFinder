#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <gazebo/msgs/msgs.hh>

//You can freely add more controllers to this list,
//but make sure you add the gazebo subscriber thingies
//for your new controller
enum class ControllerType
{
    Undefined,
    Show,
    FollowWall,
    Test
};

class Controller
{
public:
    virtual ~Controller() = 0;

    ControllerType getType();

    virtual void statCallback(ConstWorldStatisticsPtr &msg);
    virtual void poseCallback(ConstPosesStampedPtr &msg);
    virtual void cameraCallback(ConstImageStampedPtr &msg);
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg);

//protected:
    ControllerType type = ControllerType::Undefined;
};

#endif // CONTROLLER_HPP

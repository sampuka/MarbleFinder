#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

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

    virtual void statCallback();
    virtual void poseCallback();
    virtual void cameraCallback();
    virtual void lidarCallback();

//protected:
    ControllerType type = ControllerType::Undefined;
};

#endif // CONTROLLER_HPP

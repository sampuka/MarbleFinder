#ifndef PATHTRACKING_H
#define PATHTRACKING_H
#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

class PathTracking
{
public:
    PathTracking();
    void wheelAngSubscriber(ConstPosesStampedPtr &_msg);
    void draw(Mat);

    float returnPos();

private:
    float lastAngleLeft = 10;
    float lastAngleRight = 10;
    float lastTime = 0;
    float curX = 0;
    float curY = 0;
    float lastX = 0;
    float lastY = 0;
    //float position[2] = {0};
    //float lastPos[2] = {0};
    float lastOrin = 0;
};

#endif // PATHTRACKING_H

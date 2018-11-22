#ifndef MARBLELOCATOR_H
#define MARBLELOCATOR_H
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <math.h>
#include <iostream>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

using namespace cv;

class MarbleLocator
{
public:
    MarbleLocator();
    void locateMables(ConstImageStampedPtr &msg);
    bool marbleInSight();
    void centerMarble();
    float returnDir();
    float returnSpeed();
    bool isThereMoreBlue();


private:
    cv::Mat im;
    std::vector<Vec3f> circles = {0};
    std::size_t width;
    std::size_t height;
    float dir;
    float speed = 0.0;
    int max=0;
    int amountofbluee=0;
    bool isThereMoreBluepixels =false;
};

#endif // MARBLELOCATOR_H

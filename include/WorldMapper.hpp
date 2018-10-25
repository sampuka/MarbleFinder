#ifndef WORLDMAPPER_HPP
#define WORLDMAPPER_HPP

#include "Controller.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>

class WorldMapper : public Controller
{
public:
    WorldMapper();

    //virtual void statCallback(ConstWorldStatisticsPtr &msg) override;
    virtual void poseCallback(ConstPosesStampedPtr &msg) override;
    //virtual void cameraCallback(ConstImageStampedPtr &msg) override;
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg) override;

    virtual ControlOutput getControlOutput() override;

private:
    const int height = 600;
    const int width = 1000;
    const float map_height = 65;
    const float map_width = 85;
    cv::Mat worldMap;

    double x_pos = 0;
    double y_pos = 0;
    double dir = 0;
};

#endif // WORLDMAPPER_HPP

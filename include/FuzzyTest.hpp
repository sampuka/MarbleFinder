#ifndef FUZZYTEST_HPP
#define FUZZYTEST_HPP

#include "Controller.hpp"

#include "LaserScanner.hpp"
#include "fl/Headers.h"

#include <fstream>

class FuzzyTest : public Controller
{
public:
    FuzzyTest();
    virtual ~FuzzyTest() = default;

    virtual ControlOutput getControlOutput();

    //virtual void statCallback(ConstWorldStatisticsPtr &msg) override;
    //virtual void poseCallback(ConstPosesStampedPtr &msg) override;
    //virtual void cameraCallback(ConstImageStampedPtr &msg) override;
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg) override;

private:
    LaserScanner*        m_pcLaserScanner;

    fl::Engine*          m_pcFLEngine;
    fl::InputVariable*   m_pflObstacleDirection;
    fl::InputVariable*   m_pflObstacleDistance;
    fl::OutputVariable*  m_pflSteerDirection;
    fl::OutputVariable*  m_pflSpeed;

    float closest_distance = 0;
    float closest_angle = 0;

    std::ofstream time_of = std::ofstream("time.csv");
    std::ofstream dist_of = std::ofstream("distances.csv");
    std::ofstream angle_of = std::ofstream("angles.csv");
};

#endif // FUZZYTEST_HPP

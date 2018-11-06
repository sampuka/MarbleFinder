#ifndef FUZZYBUGCONTROLLER_H
#define FUZZYBUGCONTROLLER_H

/*************************************************************/
/*************************************************************/

#include "Controller.hpp"

#include "LaserScanner.hpp"
#include "fl/Headers.h"

/*************************************************************/
/*************************************************************/

class FuzzyBugController : public Controller
{
public:
    FuzzyBugController();
    virtual ~FuzzyBugController() = default;

    virtual void buildController();
    virtual ControlOutput getControlOutput();

    //virtual void statCallback(ConstWorldStatisticsPtr &msg) override;
    //virtual void poseCallback(ConstPosesStampedPtr &msg) override;
    //virtual void cameraCallback(ConstImageStampedPtr &msg) override;
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg) override;

protected:
    LaserScanner*        m_pcLaserScanner;

    fl::Engine*          m_pcFLEngine;
    fl::InputVariable*   m_pflObstacleDirection;
    fl::InputVariable*   m_pflObstacleDistance;
    fl::OutputVariable*  m_pflSteerDirection;
    fl::OutputVariable*  m_pflSpeed;
};

/*************************************************************/
/*************************************************************/

#endif // FUZZYBUGCONTROLLER_H

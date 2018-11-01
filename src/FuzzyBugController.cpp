#include "FuzzyBugController.hpp"

/*************************************************************/
/*************************************************************/

FuzzyBugController::FuzzyBugController()
{
    buildController();
    m_pcLaserScanner = new LaserScanner;
}

/*************************************************************/
/*************************************************************/

ControlOutput FuzzyBugController::getControlOutput()
{
    m_pflObstacleDistance->setValue(m_pcLaserScanner->getClosestDistance(-2.26, 2.26));
    m_pflObstacleDirection->setValue(m_pcLaserScanner->getClosestDirection(-2.26, 2.26));

    std::cout << "FL - Distance " << m_pcLaserScanner->getClosestDistance(-2.26, 2.26) << ", direction " << m_pcLaserScanner->getClosestDirection(-2.26, 2.26) << std::endl;

    m_pcFLEngine->process();

    ControlOutput out;
    out.dir       = m_pflSteerDirection->getValue();
    out.speed     = m_pflSpeed->getValue();
    std::cout<<out.dir<<std::endl;
    std::cout<<out.speed<<std::endl;
    return out;
}

/*************************************************************/
/*************************************************************/

void FuzzyBugController::buildController()
{
    using namespace fl;
    m_pcFLEngine = FllImporter().fromFile("../123.fll");
    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    m_pflObstacleDirection = m_pcFLEngine->getInputVariable("ObsDir");
    m_pflObstacleDistance  = m_pcFLEngine->getInputVariable("ObsDis");
    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("SteerDirection");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");
}

/*************************************************************/
/*************************************************************/

void FuzzyBugController::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    std::cout << "a" << std::endl;
    m_pcLaserScanner->parseLaserScannerMessage(msg);
    std::cout << "b" << std::endl;
}

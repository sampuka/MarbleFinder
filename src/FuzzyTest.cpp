#include "FuzzyTest.hpp"

#include <fstream>

FuzzyTest::FuzzyTest()
{
    m_pcFLEngine = fl::FllImporter().fromFile("../avoider.fll");
    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

    m_pflObstacleDirection = m_pcFLEngine->getInputVariable("ObsDir");
    m_pflObstacleDistance  = m_pcFLEngine->getInputVariable("ObsDis");
    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("SteerDirection");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");

    dist_of << "Distance[m]\n";
    angle_of << "Angle[rad]\n";
    time_of << "Time[ms]\n";
}

void FuzzyTest::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    //float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int nsec = msg->time().nsec();
    int sec = msg->time().sec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    float close_angle = 0;
    float close_dist = 15;

    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        //std::cout << "angle: " << angle << " - " << angle+dir << std::endl;
        float range = std::min(float(msg->scan().ranges(i)), range_max);

        if (range < close_dist)
        {
            close_dist = range;
            close_angle = angle;
        }
    }

    closest_distance = close_dist;
    closest_angle = close_angle;


    time_of << sec*1000+nsec/1000000 << '\n';
    dist_of << close_dist << '\n';
    angle_of << close_angle << '\n';

    time_of.flush();
    dist_of.flush();
    angle_of.flush();
}

ControlOutput FuzzyTest::getControlOutput()
{
    m_pflObstacleDistance->setValue(closest_distance);
    m_pflObstacleDirection->setValue(closest_angle);

    m_pcFLEngine->process();

    float speed = m_pflSpeed->getValue();
    float steer = m_pflSteerDirection->getValue();

    std::cout << "Speed: " << speed << "\tsteer: " << steer << std::endl;

    return ControlOutput{speed, steer};
}

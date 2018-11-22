#ifndef WORLDMAPPER_HPP
#define WORLDMAPPER_HPP

#include "Controller.hpp"
#include "Marblelocator.hpp"
#include <opencv2/opencv.hpp>

#include "fl/Headers.h"

#include <chrono>
#include <iostream>
#include <thread>

enum class ControllerState
{
    Exploring,
    CheckingForMarbles,
    DrivingToMarble
};

class WorldMapper : public Controller
{
public:
    WorldMapper();

    //virtual void statCallback(ConstWorldStatisticsPtr &msg) override;
    virtual void poseCallback(ConstPosesStampedPtr &msg) override;
    virtual void cameraCallback(ConstImageStampedPtr &msg) override;
    virtual void lidarCallback(ConstLaserScanStampedPtr &msg) override;

    virtual ControlOutput getControlOutput() override;

private:
    void drawlineuntil(const cv::Point &start, const cv::Point &end);

    cv::Point pos; //Position in pixels from top-left

    const int height = 600;
    const int width = 1000;
    const float map_height = 65;
    const float map_width = 85;
    cv::Mat worldMap;

    const cv::Vec3b wall_color = cv::Vec3b(0, 0, 0);
    const cv::Vec3b free_color = cv::Vec3b(255, 255, 255);
    const cv::Vec3b unknown_color = cv::Vec3b(180, 180, 180);
    const cv::Vec3b goal_color = cv::Vec3b(0, 0, 255);
    const cv::Scalar robot_color1 = cv::Scalar(0, 0, 255);
    const cv::Scalar robot_color2 = cv::Scalar(200, 0, 0);

    ControlOutput ctrlout;

    ControllerState state = ControllerState::CheckingForMarbles;

    void main_loop();
    std::thread main_loop_thread;
    const float main_loop_freq = 30; // Main loop gets run this many times per second

    void goal_update();
    std::thread goal_update_thread;
    const float goal_update_freq = 0.5;
    bool current_goal_valid = false;
    cv::Point current_goal = cv::Point(1, 0);
    std::vector<cv::Point> current_goal_path;

    float shortest_dist = 10;
    float shortest_dist_angle = 0;

    fl::Engine*          m_pcFLEngine;
    fl::InputVariable*   m_pflObstacleDirection;
    fl::InputVariable*   m_pflObstacleDistance;
    fl::OutputVariable*  m_pflSteerDirection;
    fl::OutputVariable*  m_pflSpeed;

    MarbleLocator marbl;
};

#endif // WORLDMAPPER_HPP

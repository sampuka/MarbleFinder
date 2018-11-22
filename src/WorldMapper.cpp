#include "WorldMapper.hpp"

#include "astar.hpp"
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>

WorldMapper::WorldMapper()
{
    type = ControllerType::WorldMapper;

    worldMap = cv::Mat(height, width, CV_8UC3);
    worldMap.setTo(unknown_color);

    //cv::namedWindow("lidar");
    cv::namedWindow("World Map");

    m_pcFLEngine = fl::FllImporter().fromFile("../avoider.fll");
    std::string status;
    if (not m_pcFLEngine->isReady(&status))
        throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);

    m_pflObstacleDirection = m_pcFLEngine->getInputVariable("ObsDir");
    m_pflObstacleDistance  = m_pcFLEngine->getInputVariable("ObsDis");
    m_pflSteerDirection    = m_pcFLEngine->getOutputVariable("SteerDirection");
    m_pflSpeed             = m_pcFLEngine->getOutputVariable("Speed");

    main_loop_thread = std::thread(&WorldMapper::main_loop, this);

    goal_update_thread = std::thread(&WorldMapper::goal_update, this);
}

/*
void WorldMapper::statCallback(ConstWorldStatisticsPtr &msg)
{
    (void)msg;
}
*/

void WorldMapper::poseCallback(ConstPosesStampedPtr &msg)
{
    Controller::poseCallback(msg);

    pos.x = (posf.x/map_width+0.5)*width;
    pos.y = (-posf.y/map_height+0.5)*height;
}

/*
void WorldMapper::cameraCallback(ConstImageStampedPtr &msg)
{
    (void)msg;
}
*/

void WorldMapper::drawlineuntil(const cv::Point &start, const cv::Point &end)
{
    const int step_count = 100;
    const cv::Point2f startf(start);
    const cv::Point2f endf(end);
    const cv::Point2f step((endf-startf)/step_count);

    for (int i = 0; i < step_count; i++)
    {
        cv::Point2f posf = startf+step*i;
        cv::Point posi(posf);

        if (posi == end || posi.x >= width || posi.x < 0 || posi.y >= height || posi.y < 0 || worldMap.at<cv::Vec3b>(posi) == wall_color)
            return;

        worldMap.at<cv::Vec3b>(posi) = free_color;
    }
}

void WorldMapper::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    /*
    if (msg->time().sec() == 0)
        return;
*/

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    //int sec = msg->time().sec();
    //int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int w = 400;
    int h = 400;
    float px_per_m = 200 / range_max;

    cv::Mat im(h, w, CV_8UC3);
    im.setTo(0);

    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        //std::cout << "angle: " << angle << " - " << angle+dir << std::endl;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        //    double intensity = msg->scan().intensities(i);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                 cv::LINE_AA, 4);

        //    std::cout << angle << " " << range << " " << intensity << std::endl;

        cv::Point2f mapendpt_m(posf.x + lidar_offset * std::cos(dir) + range * std::cos(angle+dir),
                               posf.y + lidar_offset * std::sin(dir) + range * std::sin(angle+dir));

        cv::Point mapendpt((mapendpt_m.x/map_width+0.5)*width,
                           (-mapendpt_m.y/map_height+0.5)*height);

        drawlineuntil(pos + cv::Point((lidar_offset * std::cos(dir)/map_width)*width,
                                      (lidar_offset * std::sin(dir)/map_height)*height),
                      mapendpt);

        //std::cout << cv::Point(x_pos, y_pos) << " " << mapendpt << std::endl;

        if (range < range_max)
        {
            worldMap.at<cv::Vec3b>(mapendpt) = wall_color;
        }
    }

    shortest_dist_angle = 0;
    shortest_dist = 999;
    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        if (range < shortest_dist)
        {
            shortest_dist = range;
            shortest_dist_angle = angle;
        }
    }

    //cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    //cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
    //           cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
    //           cv::Scalar(255, 0, 0));

    cv::Mat worldMapShow = worldMap.clone();
    //cv::Point pos((x_pos/map_width+0.5)*width, (-y_pos/map_height+0.5)*height);
    //std::cout << x_pos << ' ' << pos << std::endl;
    cv::circle(worldMapShow, pos, 5, robot_color1, -1);
    cv::circle(worldMapShow,
               pos + cv::Point((0.3 * std::cos(dir)/map_width)*width,
                               (-0.3 * std::sin(dir)/map_height)*height),
               3,
               robot_color2,
               -1);

    cv::circle(worldMapShow, current_goal, 3, goal_color);
    if (current_goal_valid)
    {
        for (const cv::Point &p : current_goal_path)
            worldMapShow.at<cv::Vec3b>(p) = goal_color;
    }

    cv::imshow("World Map", worldMapShow);
    //cv::imshow("lidar", im);
    cv::waitKey(1);
}

ControlOutput WorldMapper::getControlOutput()
{
    m_pflObstacleDirection->setValue(shortest_dist_angle);
    m_pflObstacleDistance->setValue(shortest_dist);
    m_pcFLEngine->process();
    float fl_dir = m_pflSteerDirection->getValue();
    float fl_speed = m_pflSpeed->getValue();

    const float lower = 0.7;
    const float upper = 1.2;

    float weight = (shortest_dist-lower)/(upper-lower);

    if (weight < 0)
        weight = 0;
    else if (weight > 1)
        weight = 1;

    return
    {
        ctrlout.speed*weight+(1-weight)*fl_speed,
        ctrlout.dir  *weight+(1-weight)*fl_dir
    };
}

void WorldMapper::main_loop()
{
    const std::chrono::duration<long int> marble_check_interval(8000);
    std::chrono::system_clock::time_point next_marble_check = std::chrono::system_clock::now();

    double last_dir = dir; // These two are used for turning around when finding marbles
    double total_dir = 0;

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(1/main_loop_freq));

        switch (state)
        {
        case ControllerState::Exploring:
        {
            if (std::chrono::system_clock::now() > next_marble_check)
            {
                state = ControllerState::CheckingForMarbles;
                last_dir = dir;
                total_dir = 0;
                ctrlout = ControlOutput{0, 1};
            }
            else
            {
                if (current_goal_valid)
                {
                    auto dist = [](const cv::Point &a, const cv::Point &b){

                        return std::abs(a.x-b.x)+std::abs(a.y-b.y);
                    };
                    while (current_goal_path.size() > 0 && dist(pos, current_goal_path[0]) < 40)
                        current_goal_path.erase(current_goal_path.begin());

                    float dirgoal;

                    if (current_goal_path.size() == 0)
                    {
                        dirgoal = std::atan2(current_goal.y - pos.y, current_goal.x - pos.x);
                        current_goal_valid = false;
                    }
                    else
                        dirgoal = std::atan2(current_goal_path[0].y - pos.y, current_goal_path[0].x - pos.x);

                    float direrror = dirgoal + dir;
                    if (direrror > M_PI)
                        direrror -= 2*M_PI;
                    if (direrror < - M_PI)
                        direrror += 2*M_PI;

                    //std::cout << direrror << std::endl;

                    if (direrror > 0)
                        ctrlout = ControlOutput{1, 1};
                    else
                        ctrlout = ControlOutput{1, -1};
                }
                else
                {
                    ctrlout = ControlOutput{1, 0};
                }
            }
            break;
        }
        case ControllerState::CheckingForMarbles: // cameraCallback does the actual checking
        {
            double diff_dir = last_dir-dir;
            //std::cout << "last_dir = " << last_dir << " dir = " << dir << std::endl;
            if (diff_dir < -M_PI)
                diff_dir += M_PI;

            total_dir += diff_dir;
            last_dir = dir;
            //std::cout << "diff_dir = " << diff_dir << " total_dir = " << total_dir << std::endl;
            if (total_dir > M_PI) // Found nothing
            {
                //std::cout << diff_dir << std::endl;
                state = ControllerState::Exploring;
                next_marble_check = std::chrono::system_clock::now() + marble_check_interval;
                ctrlout = ControlOutput{0, 0};
            }
            else
            {
                ctrlout = ControlOutput{0, 1};
            }
            break;
        }

        case ControllerState::DrivingToMarble:
            ctrlout = ControlOutput{1, 0};
            break;
        }
    }
}

void WorldMapper::goal_update()
{
    const std::vector<cv::Point> neighbors =
    {
        cv::Point(1 ,-1),
        cv::Point(1 , 0),
        cv::Point(1 , 1),
        cv::Point(0 , 1),
        cv::Point(-1, 1),
        cv::Point(-1, 0),
        cv::Point(-1,-1),
        cv::Point(0 ,-1)
    };

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(1/goal_update_freq));

        std::vector<cv::Point> reachable_unknowns;

        for (int i = 0; i < worldMap.rows; i++)
        {
            for (int j = 0; j < worldMap.cols; j++)
            {
                if (worldMap.at<cv::Vec3b>(i, j) == unknown_color)
                {
                    bool has_neighbor = false;

                    for (const cv::Point &p : neighbors)
                    {
                        const cv::Point neighbor = cv::Point(j, i) + p;

                        if (neighbor.x < worldMap.cols &&
                                neighbor.x >= 0 &&
                                neighbor.y < worldMap.rows &&
                                neighbor.y >= 0)
                        {
                            if (worldMap.at<cv::Vec3b>(neighbor) == free_color)
                            {
                                has_neighbor = true;
                                break;
                            }
                        }
                    }

                    if (has_neighbor)
                    {
                        reachable_unknowns.push_back(cv::Point(j, i));
                    }
                }
            }
        }

        if (reachable_unknowns.size() == 0)
        {
            std::cout << "Nothing left to explore" << std::endl;
            current_goal_valid = false;
            return;
        }

        /*
    for (const cv::Point &p : reachable_unknowns)
    {
        worldMap.at<cv::Vec3b>(p) = goal_color;
    }
*/

        const int point_range = 20;
        cv::Mat pointmap = cv::Mat(height, width, CV_32SC1);
        pointmap.setTo(0);

        for (const cv::Point &p : reachable_unknowns)
        {
            for (int i = 0; i < 2*point_range; i++)
            {
                if (i+p.x-point_range >= pointmap.cols || i+p.x-point_range < 0)
                    continue;

                for (int j = 0; j < 2*point_range; j++)
                {
                    if (j+p.y-point_range >= pointmap.rows || j+p.y-point_range < 0)
                        continue;

                    pointmap.at<std::int32_t>(j+p.y-point_range, i+p.x-point_range) += 2*point_range - std::abs(point_range-i) - std::abs(point_range-j);
                }
            }
        }

        cv::Point highest = cv::Point(0,0);
        std::int32_t value = 0;

        for (int i = 0; i < pointmap.cols; i++)
        {
            for (int j = 0; j < pointmap.rows; j++)
            {
                std::int32_t current = pointmap.at<std::int32_t>(j, i);
                //if (current != 0)
                //std::cout << current << std::endl;

                if (current > value)
                {
                    value = current;
                    highest = cv::Point(i, j);
                }
            }
        }

        //std::cout << highest << " " << value << std::endl;
        current_goal = highest;
        current_goal_valid = true;

        current_goal_path = astar(worldMap, pos, current_goal);
    }
}

#include "WorldMapper.hpp"

#include <iostream>

WorldMapper::WorldMapper()
{
    type = ControllerType::WorldMapper;

    worldMap = cv::Mat(height, width, CV_8UC3);
    worldMap.setTo(unknown_color);

    //cv::namedWindow("lidar");
    cv::namedWindow("World Map");

    main_loop_thread = std::thread(&WorldMapper::main_loop, this);
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

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int w = 400;
    int h = 400;
    float px_per_m = 200 / range_max;

    cv::Mat im(h, w, CV_8UC3);
    im.setTo(0);

    for (int i = 0; i < nranges; i++)
    //for (int i = 0; i < 1; i++)
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

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

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

    cv::imshow("World Map", worldMapShow);
    //cv::imshow("lidar", im);
    cv::waitKey(1);
}

ControlOutput WorldMapper::getControlOutput()
{
    return ctrlout;
}

void WorldMapper::main_loop()
{

    const std::chrono::duration<long int> marble_check_interval(8);
    std::chrono::system_clock::time_point next_check = std::chrono::system_clock::now();

    double last_dir = dir; // These two are used for turning around when finding marbles
    double total_dir = 0;

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(1/main_loop_freq));

        switch (state)
        {
        case ControllerState::Exploring:
            if (std::chrono::system_clock::now() > next_check)
            {
                state = ControllerState::CheckingForMarbles;
                last_dir = dir;
                total_dir = 0;
                ctrlout = ControlOutput{0, 1};
            }
            else
            {
                ctrlout = ControlOutput{1, 0};
            }
            break;

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
                next_check = std::chrono::system_clock::now() + marble_check_interval;
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

        case ControllerState::ReturningToPos:
            ctrlout = ControlOutput{1, 0};
            break;

        case ControllerState::ReturningToDir:
            ctrlout = ControlOutput{0, 0.5};
            break;
        }
    }
}
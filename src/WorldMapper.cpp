#include "WorldMapper.hpp"

#include <iostream>

WorldMapper::WorldMapper()
{
    type = ControllerType::WorldMapper;

    worldMap = cv::Mat(height, width, CV_8UC3);
    worldMap.setTo(cv::Scalar(255, 255, 255));

    cv::namedWindow("lidar");
    cv::namedWindow("World Map");
}

/*
void WorldMapper::statCallback(ConstWorldStatisticsPtr &msg)
{
    (void)msg;
}
*/

void WorldMapper::poseCallback(ConstPosesStampedPtr &msg)
{
    int pose_size = msg->pose_size();

    for (int i = 0; i < pose_size; i++)
    {
        if (msg->pose(i).name() == "pioneer2dx")
        {
            x_pos = msg->pose(i).position().x();
            y_pos = msg->pose(i).position().y();

            //dir = msg->pose(i).orientation().z()*2; //Not confiremd it is z
            double w = msg->pose(i).orientation().w();
            double x = msg->pose(i).orientation().x();
            double y = msg->pose(i).orientation().y();
            double z = msg->pose(i).orientation().z();

            dir = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
            // Magic from Wikipedia
            // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        }
    }

    std::cout << std::setprecision(2) << std::fixed <<
                 "x_pos = " << std::setw(6) << x_pos <<
                 "y_pos = " << std::setw(6) << y_pos <<
                 "dir = "   << std::setw(6) << dir   << std::endl;
}

/*
void WorldMapper::cameraCallback(ConstImageStampedPtr &msg)
{
    (void)msg;
}
*/

void WorldMapper::lidarCallback(ConstLaserScanStampedPtr &msg)
{
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

        if (range < range_max-0.1 && sec > 0)
        {
            //cv::Point mapendpt(((endpt.y/20)+0.5)*height, ((endpt.x/20)+0.5)*width);
            //cv::Point mapendpt(endpt);
            cv::Point2f mapendpt_m(x_pos + range * std::cos(angle+dir), y_pos + range * std::sin(angle+dir));
            cv::Point mapendpt((mapendpt_m.x/map_width+0.5)*width,
                               (-mapendpt_m.y/map_height+0.5)*height);

            std::cout << range << std::endl;
            std::cout << mapendpt << std::endl;
            //cv::circle(worldMap, mapendpt, 1, cv::Scalar(0, 0, 255, 255), -1);
            worldMap.at<cv::Vec3b>(cv::Point(mapendpt.x, mapendpt.y)) = cv::Vec3b(0, 0, 0);
        }
    }

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));

    cv::Mat worldMapShow = worldMap.clone();
    cv::Point pos((x_pos/map_width+0.5)*width, (-y_pos/map_height+0.5)*height);
    //std::cout << x_pos << ' ' << pos << std::endl;
    cv::circle(worldMapShow, pos, 5, cv::Scalar(0, 0, 255, 255), -1);

    cv::imshow("World Map", worldMapShow);
    cv::imshow("lidar", im);
    cv::waitKey(1);
}

ControlOutput WorldMapper::getControlOutput()
{
    return ControlOutput{1, 0};
}

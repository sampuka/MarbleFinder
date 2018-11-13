#include <iomanip>
#include <math.h>
#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

/* void locateMables(ConstImageStampedPtr &msg){

    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
   // cvtColor(im, im, CV_BGR2RGB);

    Mat convIm = im.clone();
    //cv::cvtColor(im ,convIm, CV_RGB2HLS);

    cvtColor(convIm ,convIm, CV_BGR2GRAY);                             // convert colour to gray
    //GaussianBlur( convIm, convIm, cv::Size(9, 9), 2, 2 );               //add blur
    std::vector<Vec3f> circles;                                       //vector to store the circles detected

    HoughCircles( convIm, circles, CV_HOUGH_GRADIENT, 1, convIm.rows/8, 200, 20, 0, 0 );

    cvtColor(im ,im, CV_BGR2RGB);

    for( size_t i = 0; i < circles.size(); i++ )
      {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( im, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( im, center, radius, Scalar(0,0,255), 1, 8, 0 );
       }


    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();

    mutex.lock();
    cv::imshow("My camera", convIm);
    mutex.unlock();
}*/

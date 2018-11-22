#include "Marblelocator.hpp"
#include <iostream>


using namespace cv;

MarbleLocator::MarbleLocator()
{

}

void MarbleLocator::locateMables(ConstImageStampedPtr &msg){
    static boost::mutex mutex;

    width = msg->image().width();
    height = msg->image().height();
    const char *data = msg->image().data().c_str();
    im = Mat(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();

    Mat convIm = im.clone();

    cvtColor(convIm ,convIm, CV_BGR2HLS);



    for(int i = 0; i< convIm.rows; i++){
         for(int j = 0; j < convIm.cols; j++){

            uchar H = convIm.at<Vec3b>(i,j)[0];
            uchar L = convIm.at<Vec3b>(i,j)[1];
            uchar S = convIm.at<Vec3b>(i,j)[2];

            double LS_ratio = ((double) L) / ((double) S);
            bool blue_pixel = (S >= 90) && (LS_ratio > 0.5) &&
                  (LS_ratio < 3.0) && ((H <= 250) && (H >= 200));

            if(!blue_pixel){
                convIm.at<Vec3b>(i,j)[2] = 0;

             }

          }
       }

    cvtColor(convIm ,convIm, CV_HLS2BGR);

    cvtColor(convIm ,convIm, CV_BGR2GRAY);                             // convert colour to gray
    //GaussianBlur( convIm, convIm, cv::Size(9, 9), 2, 2 );            //add blur

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

        std::cout << "blue pixel: " << im.at<cv::Vec3b>(center) << std::endl;
    }
    isThereMoreBlue();

    if(!isThereMoreBluepixels)
        centerMarble();


    mutex.lock();
    cv::imshow("My camera", im);
    mutex.unlock();

   // std::cout<<cvRound(circles[0][0])<<std::endl;
}

bool MarbleLocator::marbleInSight(){
    if (circles.size() != 0 ||isThereMoreBluepixels){
        return true;
    }

   return false;
}

void MarbleLocator::centerMarble(){
    int closest = 0;
    int closMarble = 0;
    for (unsigned int i = 0; i < circles.size(); i++ ){
        if(circles[i][2] > closest){
            closest = circles[i][2];
            closMarble = i;
        }
    }

    int imageCenter = 320.0/2;
    int center = cvRound(circles[closMarble][0]);
    if (!marbleInSight() || closest < 5){
        dir = 0.3;
        speed = 0.1;

    }
    else{
        if (center>imageCenter+10){
            dir = 0.3;
            speed = 0.2;
        }
        if (center<imageCenter-10){
            dir = -0.3;
            speed = 0.2;
        }
        if(center<imageCenter+10 && center>imageCenter-10){
            dir = 0.0;
            speed = 0.5;
        }
    }
}

float MarbleLocator::returnDir(){
    return dir;
}

float MarbleLocator::returnSpeed(){
    return speed;
}

bool MarbleLocator::isThereMoreBlue(){
    max=im.rows*im.cols;
    Mat convIm = im.clone();
    amountofbluee=0;


    for(int i = 0; i< convIm.rows; i++){
        for(int j = 0; j < convIm.cols; j++){

            uchar B = im.at<Vec3b>(i,j)[0];
            uchar G = im.at<Vec3b>(i,j)[1];
            uchar R = im.at<Vec3b>(i,j)[2];

            //double LS_ratio = ((double) L) / ((double) S);
            //bool blue_pixel = (S >= 90) && (LS_ratio > 0.5) &&
                    //(LS_ratio < 3.0) && ((H <= 250) && (H >= 200));
            bool blue_pixel = B>10 && G < 10 && R < 10;

            if(!blue_pixel){
                convIm.at<Vec3b>(i,j)[2] = 0;

            }
            if(blue_pixel)
            {
                amountofbluee++;
            }

        }
    }

    std::cout << amountofbluee << std::endl;


    if(amountofbluee>max/10)
    {

        isThereMoreBluepixels=true;
        dir=0.0;
        speed=1.0;
    }
    if(amountofbluee<max/10)
        isThereMoreBluepixels=false;

    return isThereMoreBluepixels;
}

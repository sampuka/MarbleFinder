#ifndef BRUSHFIRE_H
#define BRUSHFIRE_H

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "iostream"
#include "vector"
using namespace cv;
using namespace std;
class Brushfire
{
public:
    Brushfire();
    void makeBrushfire();
    void showBrushfire();


    ~Brushfire();
private:
    Mat getimage();
    Mat image;
    Mat image2;
};

#endif // BRUSHFIRE_H

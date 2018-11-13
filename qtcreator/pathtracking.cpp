#include "pathtracking.h"
#include <iomanip>
#include <math.h>
#include <limits>
#define PI 3.14159265359

PathTracking::PathTracking()
{
    lastX = 0;
    lastY = 0;
    curX = 0;
    curY = 0;
}

void PathTracking::wheelAngSubscriber(ConstPosesStampedPtr &_msg)
{
    float angleRight;
    float angleLeft;
    float time;
    float velocityRight;
    float velocityLeft;
    float orientation;
    float deltaAngRight;
    float deltaAngLeft;
    float deltaTime;

    float centToICC;
    float lengthRobot = 0.39;
    float wheelRadius = 0.11;
    float omegaICC;


   // std::cout << _msg->DebugString();
    int sec = _msg->time().sec();
    int nsec = _msg->time().nsec();

    time = sec+nsec/pow(10,9);

    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::right_wheel") {
          double w1 = _msg->pose(i).orientation().w();
          double x1 = _msg->pose(i).orientation().x();
          double y1 = _msg->pose(i).orientation().y();
          double z1 = _msg->pose(i).orientation().z();

          double w = 0.5*(w1-x1-y1-z1);
          double y = 0.5*(w1-x1+y1+z1);

          angleRight = -atan2(w,y);
      }
      if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::left_wheel") {
          double w1 = _msg->pose(i).orientation().w();
          double x1 = _msg->pose(i).orientation().x();
          double y1 = _msg->pose(i).orientation().y();
          double z1 = _msg->pose(i).orientation().z();

          double w = 0.5*(w1-x1-y1-z1);
          double y = 0.5*(w1-x1+y1+z1);

          angleLeft = -atan2(w,y);
      }
    }

    if(lastAngleLeft == 10 || lastAngleRight == 10){
   //     lastAngleLeft = angleLeft;
   //     lastAngleRight = angleRight;
    }


    deltaAngRight = angleRight-lastAngleRight;
    deltaAngLeft = angleLeft-lastAngleLeft;
    deltaTime = time-lastTime;
    lastTime = time;

    // next part handles when the wheel passes 180 degrees

    if(deltaAngRight > PI){
        deltaAngRight -= 2*PI;
    }
    else if(deltaAngRight < -PI){
        deltaAngRight += 2*PI;
    }

    if(deltaAngLeft > PI){
        deltaAngLeft -= 2*PI;
    }
    else if(deltaAngLeft < -PI){
        deltaAngLeft += 2*PI;
    }

    // the velocity of the wheels

    velocityRight = 2*wheelRadius*(deltaAngRight)/(deltaTime);
    velocityLeft = 2*wheelRadius*(deltaAngLeft)/(deltaTime);

    lastAngleRight = angleRight;
    lastAngleLeft = angleLeft;

    if(velocityLeft == velocityRight){
        centToICC = 0;
   }else{
    centToICC = (lengthRobot/2) * (velocityLeft + velocityRight)/(velocityRight - velocityLeft);
    }
    omegaICC = (velocityRight - velocityLeft)/lengthRobot;
    orientation = omegaICC*deltaTime + lastOrin;

    float iccx = lastX-centToICC*sin(lastOrin);
    float iccy = lastY+centToICC*cos(lastOrin);

    curX= cos(omegaICC*deltaTime)*(lastX-iccx)-sin(omegaICC*deltaTime)*(lastY-iccy)+iccx;
    curY= sin(omegaICC*deltaTime)*(lastX-iccx)+cos(omegaICC*deltaTime)*(lastY-iccy)+iccy;


   // std::cout<<std::fixed <<std::setw(6)<<velocityLeft<<" - "<<std::setw(6)<<velocityRight
     //    <<" - "<<std::setw(6)<<curX<<" - "<<std::setw(6)<<curY<<"-"<<std::setw(6)<<orientation<<std::endl;


    lastOrin = orientation;
    lastX = curX;
    lastY = curY;

}

void PathTracking::draw(Mat scaledMap)
{
    double  scalar = 72/25.4 * 0.5*5;

    Point2f dirVector(cos(lastOrin), sin(-lastOrin));
    Point2f pos(scalar*lastX + scaledMap.cols/2 , scalar*lastY + scaledMap.rows/2);

    Mat displayMap;
    scaledMap.copyTo(displayMap);
    arrowedLine(displayMap,pos,pos+5*dirVector,Scalar(0,0,255),1,8,0,0.5);

    imshow("DisplayMap",displayMap);


}

float PathTracking::returnPos(){
    return 2.1;
}

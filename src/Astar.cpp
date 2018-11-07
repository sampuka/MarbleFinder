#include "Astar.hpp"
using namespace cv;
using namespace std;
Astar::Astar()
{

}

struct cell
{
    int parent_i,parent_j;
    double f,g,h;
};

bool isValid(const Mat &map, const Point &xpoint)
{
 int ROWS =map.rows;
 int COLS = map.cols;
return(xpoint.x>=0 && xpoint.x<COLS && xpoint.y>=0 && xpoint.y<ROWS);

}

bool isUnblocked(const Mat &map,const Point &xpoint)
{
    if(map.at<Vec3b>(xpoint.y,xpoint.x)!=Vec3b(0,0,0))
    return(true);
    else
    return(false);
}

bool isDestination(const Mat &map, const Point &cpoint,const  Point &epoint)
{
    if(map.at<Vec3b>(cpoint) == map.at<Vec3b>(epoint))
        return true;
    else
        return false;

}

double calculateHValue(Mat &map, Point cpoint, Point &epoint)
{
    return((double)sqrt ((cpoint.x-epoint.x)*(cpoint.y-epoint.y)));
}


std::vector<cv::Point> astar(const cv::Mat &map, const cv::Point &start, const cv::Point &end)
{
    typedef pair<int,Point> pPair;
  vector<Point> emptyvec;
int rows =map.rows;
int cols =map.cols;
if(isValid(map,start)==false && isValid(map,end)==false)
{
    return emptyvec;
}
if(isUnblocked(map,start)==false || isUnblocked(map,end))
   {
    return emptyvec;
   }
if(isDestination(map, start,end)== true)
{
    return emptyvec;
}
bool closedList [rows][cols];
memset(closedList,false,sizeof(closedList));
int i, j;
cell cellDetails[rows][cols];
for(i=0;i<rows;i++)
{
    for(j=0;j<cols; j++)
    {
       cellDetails[i][j].f=FLT_MAX;
       cellDetails[i][j].g=FLT_MAX;
       cellDetails[i][j].h=FLT_MAX;
       cellDetails[i][j].parent_i=-1;
       cellDetails[i][j].parent_j=-1;
    }
}

i=start.y;
j=start.x;
cellDetails[i][j].f=0.0;
cellDetails[i][j].g=0.0;
cellDetails[i][j].h=0.0;
cellDetails[i][j].parent_i=i;
cellDetails[i][j].parent_j=j;

set<pPair> openList;

openList.insert(make_pair(0.0,Point(i,j)));

/*
 Generating all the 8 successor of this cell

     N.W   N   N.E
       \   |   /
        \  |  /
     W----Cell----E
          / | \
        /   |  \
     S.W    S   S.E

 Cell-->Popped Cell (i, j)
 N -->  North       (i-1, j)
 S -->  South       (i+1, j)
 E -->  East        (i, j+1)
 W -->  West           (i, j-1)
 N.E--> North-East  (i-1, j+1)
 N.W--> North-West  (i-1, j-1)
 S.E--> South-East  (i+1, j+1)
 S.W--> South-West  (i+1, j-1)*/



}

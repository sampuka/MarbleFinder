#include "Astar.hpp"
using namespace cv;
using namespace std;


struct cell
{
    int parent_i,parent_j;
    double f,g,h;
};

bool isValid(const Mat &map,const Point &xpoint)
{
   int ROWS =map.rows;
   int COLS = map.cols;
   return(xpoint.x>=0 && xpoint.x<COLS && xpoint.y>=0 && xpoint.y<ROWS);

}

bool isUnblocked(const Mat &map,const Point &xpoint)
{
    if(map.at<Vec3b>(xpoint.y,xpoint.x)==Vec3b(255,255,255))
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

double calculateHValue( const Point &cpoint,  const Point &epoint)
{
    return((double)sqrt ((cpoint.x-epoint.x)*(cpoint.y-epoint.y)));
}

vector<Point> tracePath(vector<vector<cell>> cellDetails, const Point &xpoint)
{
   int row= xpoint.y;
   int col=xpoint.x;
   vector<Point> Pathvec;
   stack<Point> Path;

    while (!(cellDetails[row][col].parent_i == row
                && cellDetails[row][col].parent_j == col ))
        {
            Path.push (Point(row, col));
            int temp_row = cellDetails[row][col].parent_i;
            int temp_col = cellDetails[row][col].parent_j;
            row = temp_row;
            col = temp_col;
        }
    Path.push(Point(row,col));
    while(!Path.empty())
    {
        Point P= Path.top();
        Path.pop();
        Pathvec.push_back(P);
    }
return Pathvec;

}


std::vector<cv::Point> astar(const cv::Mat &map, const cv::Point &start,const  cv::Point &end)
{
    typedef pair<int,Point> pPair;
    vector<Point> emptyvec;
    int rows =map.rows;
    int cols =map.cols;
    if(!isValid(map,start) && !isValid(map,end))
    {
        return emptyvec;
    }
    if(isUnblocked(map,start)==false || isUnblocked(map,end)==true)
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
    vector<vector<cell>> cellDetails;
    for(i=0;i<rows;i++)
    {
        cellDetails.push_back(vector<cell>());
        for(j=0;j<cols; j++)
        {
            cell newcell;
           newcell.f=FLT_MAX;
           newcell.g=FLT_MAX;
           newcell.h=FLT_MAX;
           newcell.parent_i=-1;
           newcell.parent_j=-1;
           cellDetails[i].push_back(newcell);
        }
    }

    i=start.y;
    j=start.x;
    cellDetails[i][j].f=0.0;
    cellDetails[i][j].g=0.0;
    cellDetails[i][j].h=0.0;
    cellDetails[i][j].parent_i=i;
    cellDetails[i][j].parent_j=j;

    vector<pPair> openList;

    openList.push_back(make_pair(0.0,Point(i,j)));

    bool foundDest =false;

    while(!openList.empty())
    {
        pPair p=*openList.begin();

        openList.erase(openList.begin());
        i=p.second.y;
        j=p.second.x;
        closedList[i][j]=true;
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

    double gNew, hNew, fNew;


            for(int z=-1;z<1;z++)
            {
                for(int k=-1; k<1;k++)
                {
                    if(z==0 && k==0)
                    {
                        continue;
                    }
                        if(isValid(map,Point(i-z,j-k))==true)
                        {
                        if(isDestination(map,Point(i-z,j-k),end)== true)
                        {
                            cellDetails[i-z][j-k].parent_i=i;
                            cellDetails[i-z][j-k].parent_j=j;
                            foundDest= true;
                           return tracePath(cellDetails,end);

                        }
                        else if (closedList[i-z][j-k]==false && isUnblocked(map,Point(i,j)))
                       {
                            gNew=cellDetails[i][j].g+1.0;
                            hNew=calculateHValue(Point(i-1,j),end);
                            fNew=gNew+hNew;

                       if (cellDetails[i-z][j-k].f>fNew)
                       {
                           openList.push_back(make_pair(fNew,Point(i-z,j-k)));
                           cellDetails[i-z][j-k].f=fNew;
                           cellDetails[i-z][j-k].g=gNew;
                           cellDetails[i-z][j-k].h=hNew;
                           cellDetails[i-z][j-k].parent_i=i;
                           cellDetails[i-z][j-k].parent_j=j;
                       }
                    }

                }
            }
        }

            if(foundDest==false)
            return emptyvec;

    }
}


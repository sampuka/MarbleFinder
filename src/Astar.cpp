#include "Astar.hpp"
using namespace cv;
using namespace std;


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

double calculateHValue(const Mat &map, const Point cpoint,  const Point &epoint)
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

    set<pPair> openList;

    openList.insert(make_pair(0.0,Point(i,j)));

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
    if(isValid(map,Point(i-1,j))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i-1,j),end)== true)
        {
            cellDetails[i-1][j].parent_i=i;
            cellDetails[i-1][j].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i-1][j]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i-1,j),end);
           fNew=gNew+hNew;

           if (cellDetails[i-1][j].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i-1,j)));
               cellDetails[i-1][j].f=fNew;
               cellDetails[i-1][j].g=gNew;
               cellDetails[i-1][j].h=hNew;
               cellDetails[i-1][j].parent_i=i;
               cellDetails[i-1][j].parent_j=j;
           }
        }
    }

    //2 sucessor south
    if(isValid(map,Point(i+1,j))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i+1,j),end)== true)
        {
            cellDetails[i+1][j].parent_i=i;
            cellDetails[i+1][j].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i+1][j]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i+1,j),end);
           fNew=gNew+hNew;

           if (cellDetails[i+1][j].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i+1,j)));
               cellDetails[i+1][j].f=fNew;
               cellDetails[i+1][j].g=gNew;
               cellDetails[i+1][j].h=hNew;
               cellDetails[i+1][j].parent_i=i;
               cellDetails[i+1][j].parent_j=j;
           }
        }
    }
// 3 sicesspr east
    if(isValid(map,Point(i,j+1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i,j+1),end)== true)
        {
            cellDetails[i][j+1].parent_i=i;
            cellDetails[i][j+1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i][j+1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i,j+1),end);
           fNew=gNew+hNew;

           if (cellDetails[i][j+1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i,j+1)));
               cellDetails[i][j+1].f=fNew;
               cellDetails[i][j+1].g=gNew;
               cellDetails[i][j+1].h=hNew;
               cellDetails[i][j+1].parent_i=i;
               cellDetails[i][j+1].parent_j=j;
           }
        }
    }
// 4 sucessor
    if(isValid(map,Point(i,j-1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i,j-1),end)== true)
        {
            cellDetails[i][j-1].parent_i=i;
            cellDetails[i][j-1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i][j-1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i,j-1),end);
           fNew=gNew+hNew;

           if (cellDetails[i][j-1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i,j-1)));
               cellDetails[i][j-1].f=fNew;
               cellDetails[i][j-1].g=gNew;
               cellDetails[i][j-1].h=hNew;
               cellDetails[i][j-1].parent_i=i;
               cellDetails[i][j-1].parent_j=j;
           }
        }
    }
    //5 sucessor north east
    if(isValid(map,Point(i-1,j+1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i-1,j+1),end)== true)
        {
            cellDetails[i-1][j+1].parent_i=i;
            cellDetails[i-1][j+1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i-1][j+1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i-1,j+1),end);
           fNew=gNew+hNew;

           if (cellDetails[i-1][j+1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i-1,j+1)));
               cellDetails[i-1][j+1].f=fNew;
               cellDetails[i-1][j+1].g=gNew;
               cellDetails[i-1][j+1].h=hNew;
               cellDetails[i-1][j+1].parent_i=i;
               cellDetails[i-1][j+1].parent_j=j;
           }
        }
    }
    //6 sucessor north west
    if(isValid(map,Point(i-1,j-1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i-1,j-1),end)== true)
        {
            cellDetails[i-1][j-1].parent_i=i;
            cellDetails[i-1][j-1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i-1][j-1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i-1,j-1),end);
           fNew=gNew+hNew;

           if (cellDetails[i-1][j-1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i-1,j-1)));
               cellDetails[i-1][j-1].f=fNew;
               cellDetails[i-1][j-1].g=gNew;
               cellDetails[i-1][j-1].h=hNew;
               cellDetails[i-1][j-1].parent_i=i;
               cellDetails[i-1][j-1].parent_j=j;
           }
        }
    }
    // 7 sucessor south east
    if(isValid(map,Point(i+1,j+1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i+1,j+1),end)== true)
        {
            cellDetails[i+1][j+1].parent_i=i;
            cellDetails[i+1][j+1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i+1][j+1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i+1,j+1),end);
           fNew=gNew+hNew;

           if (cellDetails[i+1][j+1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i+1,j+1)));
               cellDetails[i+1][j+1].f=fNew;
               cellDetails[i+1][j+1].g=gNew;
               cellDetails[i+1][j+1].h=hNew;
               cellDetails[i+1][j+1].parent_i=i;
               cellDetails[i+1][j+1].parent_j=j;
           }
        }
    }
    // 8 sucessor south west
    if(isValid(map,Point(i+1,j-1))==true)
    {
        //1st Sucessor North
        if(isDestination(map,Point(i+1,j-1),end)== true)
        {
            cellDetails[i+1][j-1].parent_i=i;
            cellDetails[i+1][j-1].parent_j=j;
            foundDest= true;
            return tracePath(cellDetails,end);

        }
        else if (closedList[i+1][j-1]==false && isUnblocked(map,Point(i,j)))
        {
           gNew=cellDetails[i][j].g+1.0;
           hNew=calculateHValue(map,Point(i+1,j-1),end);
           fNew=gNew+hNew;

           if (cellDetails[i+1][j-1].f>fNew)
           {
               openList.insert(make_pair(fNew,Point(i-1,j)));
               cellDetails[i+1][j-1].f=fNew;
               cellDetails[i+1][j-1].g=gNew;
               cellDetails[i+1][j-1].h=hNew;
               cellDetails[i+1][j-1].parent_i=i;
               cellDetails[i+1][j-1].parent_j=j;
           }
        }
    }
    if(foundDest==false)
        return emptyvec;
}
}

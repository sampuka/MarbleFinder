#include "astar.hpp"

#include <mutex>
#include<unordered_set>
#include<algorithm>

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
    if(map.at<Vec3b>(xpoint.y,xpoint.x)!=Vec3b(0,0,0))
        return(true);
    else
        return(false);
}

bool isDestination(const Point &cpoint,const  Point &epoint)
{
    if(cpoint.x == epoint.x && cpoint.y ==epoint.y)
        return true;
    else
        return false;

}

double calculateHValue(int row, int col, const Point &dest)
{
    //float dist = abs(col-dest.x)+abs(row-dest.y);
    float dist = sqrt((row-dest.y)*(row-dest.y) +
                (col-dest.x)*(col-dest.x));

    //std::cout << dist << ' ' << sqrt((row-dest.y)*(row-dest.y) + (col-dest.x)*(col-dest.x)) << std::endl;

    return dist;
}

vector<cv::Point> tracePath(vector<vector<cell>> cellDetails, const Point &xpoint)
{
    int row= xpoint.x;
    int col=xpoint.y;
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
        //cout<<P<<endl;

        Path.pop();
        Pathvec.push_back(P);
    }


    return Pathvec;
}

vector<Point> astar(const cv::Mat &_map, const cv::Point &_start, const cv::Point &_end)
{
    mutex mtx;

    const cv::Mat map = _map.clone();
    const cv::Point start = _start;
    const cv::Point end = _end;

    typedef pair<double,Point> pPair;
    vector<Point> resvec;
    vector<Point> emptyvec;
    int rows =map.rows;
    int cols =map.cols;
    if(!isValid(map,start) && !isValid(map,end))
    {cout<<"istn a valid point"<<endl;
        return emptyvec;
    }
    if(isUnblocked(map,start)==false || isUnblocked(map,end)==false)
    {cout<<"is blocked"<<endl;
        return emptyvec;
    }
    if(isDestination(start,end)== true)
    {
        cout<<" is desitnation"<<endl;
        return emptyvec;

    }
    bool closedList [cols][rows];
    memset(closedList,false,sizeof(closedList));
    int i, j;
    vector<vector<cell>> cellDetails;
    for(i=0;i<cols;i++)
    {
        cellDetails.push_back(vector<cell>());
        for(j=0;j<rows; j++)
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

    i=start.x;
    j=start.y;
    cellDetails[i][j].f=0.0;
    cellDetails[i][j].g=0.0;
    cellDetails[i][j].h=0.0;
    cellDetails[i][j].parent_i=i;
    cellDetails[i][j].parent_j=j;

    auto compare = [](pair<double,Point> a, pair<double,Point> b){return a.first < b.first;};
    set<pair<double,Point>, decltype(compare)> openList(compare);

    mtx.lock();
    openList.insert((make_pair<double,Point>(0.0,Point(i,j))));
    mtx.unlock();

    bool foundDest =false;

    while(!openList.empty())
    {
        pPair p=*openList.begin();
        mtx.lock();
        openList.erase(openList.begin());
        mtx.unlock();
        i=int(p.second.x);
        j=int(p.second.y);
        closedList[i][j]=true;


        double gNew, hNew, fNew;


        for(int z=-1;z<2;z++)
        {
            for(int k=-1; k<2;k++)
            {
                if(z==0 && k==0)
                {
                    continue;
                }
                if(isValid(map,Point(i-z,j-k))==true)
                {
                    if(isDestination(Point(i-z,j-k),end)== true)
                    {
                        cellDetails[i-z][j-k].parent_i=i;
                        cellDetails[i-z][j-k].parent_j=j;
                        //cout<<"Destination is found"<<endl;;
                        foundDest= true;
                        resvec =tracePath(cellDetails,end);
                        openList.clear();
                    }
                    else if (closedList[i-z][j-k]==false && isUnblocked(map,Point(i-z,j-k))==true)
                    {
                        if (z*k != 0)
                            gNew=cellDetails[i][j].g+1.5;
                        else
                            gNew=cellDetails[i][j].g+1.0;

                        hNew=calculateHValue(i-z,j-k,Point(end));
                        fNew=gNew+hNew;

                        if (cellDetails[i-z][j-k].f>fNew)
                        {
                            mtx.lock();
                            openList.insert(pair<double,Point>(fNew,Point(i-z,j-k)));
                            mtx.unlock();
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
    }
    if(foundDest==false)
    {
        cout<<"no destination found"<<endl;
        resvec.clear();
    }
    return resvec;
}


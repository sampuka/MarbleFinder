#include "dijkstra.hpp"

#include <iostream>

typedef struct
{
    bool checked = false;
    cv::Point parent;
    double dist = std::numeric_limits<float>::max();
} node;

const std::vector<cv::Point> neighbors =
{
    cv::Point(1 , 1),
    cv::Point(0 , 1),
    cv::Point(-1, 1),
    cv::Point(-1, 0),
    cv::Point(-1,-1),
    cv::Point(0 ,-1),
    cv::Point(1 ,-1),
    cv::Point(1 , 0)
};

bool isInside(const cv::Mat &map, const cv::Point &p)
{
    return (p.x>=0) && (p.x<map.cols) && (p.y>=0) && (p.y<map.rows);
}

bool isBlocked(const cv::Mat &map, const cv::Point &p)
{
    return map.at<cv::Vec3b>(p) == cv::Vec3b(0, 0, 0);
}

double dist(const cv::Point &a, const cv::Point &b)
{
    return std::sqrt(std::pow(a.x-b.x, 2)+std::pow(a.x-b.x,2));
}

std::vector<cv::Point> findPath(const std::vector<std::vector<node>> &grid, const cv::Point &start, const cv::Point &end)
{
    std::vector<cv::Point> path = {end};
    cv::Point current = grid[end.x][end.y].parent;
    while (current != start)
    {
        std::cout << current << std::endl;
        path.push_back(current);
        current = grid[current.x][current.y].parent;
    }

    return path;
}

std::vector<cv::Point> dijkstra(const cv::Mat &_map, const cv::Point &_start, const cv::Point &_end)
{
    std::cout << "dijkstra called " << std::endl;
    const cv::Mat map = _map.clone();
    const cv::Point start = _start;
    const cv::Point end = _end;

    if (!isInside(map, start) || !isInside(map, end))
        std::cout << "Invalid points" << std::endl;

    auto comp = [end](const cv::Point &a, const cv::Point &b){return dist(a, end) < dist(b, end);};

    std::set<cv::Point, decltype(comp)> openList(comp);
    openList.insert(start);

    //std::vector<cv::Point> openList;
    //openList.push_back(start);

    std::vector<std::vector<node>> grid;

    grid.resize(map.cols);

    for (auto &row : grid)
        row.resize(map.rows);

    grid[start.x][start.y].dist = 0;

    std::vector<cv::Point> path;

    while (!openList.empty())
    {
        const cv::Point current = *openList.begin();
        openList.erase(openList.begin());

        std::cout << current << std::endl;

        grid[current.x][current.y].checked = true;

        for (const cv::Point &p : neighbors)
        {
            const cv::Point neighbor = current+p;

            if (neighbor == end)
            {
                grid[neighbor.x][neighbor.y].parent = current;
                grid[neighbor.x][neighbor.y].dist = grid[current.x][current.y].dist + dist(current, neighbor);
                openList.clear();
            }

            else if (!isInside(map, neighbor) || isBlocked(map, neighbor))
                continue;

            else if (grid[neighbor.x][neighbor.y].checked)
            {
                double start_dist = dist(current,neighbor)+grid[neighbor.x][neighbor.y].dist;

                if (start_dist < grid[current.x][current.y].dist)
                {
                    grid[current.x][current.y].dist = start_dist;
                    grid[current.x][current.y].parent = neighbor;
                }
            }
            else
                openList.insert(neighbor);
        }
    }

    std::cout << grid[start.x+1][start.x+1].dist << std::endl;

    return path;
}

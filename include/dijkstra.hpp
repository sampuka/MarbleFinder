#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <opencv2/opencv.hpp>

#include <vector>

std::vector<cv::Point> dijkstra(const cv::Mat &map, const cv::Point &start, const cv::Point &end);

#endif // DIJKSTRA_H

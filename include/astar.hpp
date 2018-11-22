#ifndef ASTAR_HPP
#define ASTAR_HPP
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<vector>
#include<bits/stdc++.h>
#include<set>
std::vector<cv::Point> astar(const cv::Mat &map, const cv::Point &start, const cv::Point &end);
#endif // ASTAR_HPP

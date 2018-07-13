#ifndef RISK_MAP_H
#define RISK_MAP_H
#include "ros/ros.h"
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>
using namespace std;
class risk_map{
  public:
    int rows;
    int cols;
    double mapResolution;
    vector<vector <bool> > grid;
    nav_msgs::OccupancyGrid my_map;
    void readMap(const nav_msgs::OccupancyGrid& map);
    bool requestMap(ros::NodeHandle &nh);
    void printGrid();
};

#endif

#ifndef RISK_GRAPH_H
#define RISK_GRAPH_H
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
using namespace std;

class risk_barrier{
  public:
    int x; //center pose (x,y)
    int y;
    int r; //radius
};
//-----------------------------------------------------------------------

class risk_node{
  public:
    int x;
    int y;
    char parent;
    double edge_cost[4];//order 0-3:URDL or CW
    risk_node();
    void edge_cost_calculation(risk_barrier barriers[12]);
};
//-----------------------------------------------------------------------

class risk_path_cost_info{
  public:
    string path;
    double path_cost;
    double max_path_cost;
    risk_path_cost_info();
};
//-----------------------------------------------------------------------

class risk_graph{
  public:
    int num_of_barriers;
    risk_barrier *barriers;
    int cols_l;
    int rows_l;
    risk_node* graph_nodes;
    int V;
    double *dist;
    bool *sptSet;
    double resolution;
    float bias[3];
    nav_msgs::OccupancyGrid costmap;
    nav_msgs::OccupancyGrid local_costmap;
    risk_path_cost_info path;
    geometry_msgs::PoseStamped rviz_pose;

    risk_path_cost_info robot_decision_info;
    risk_path_cost_info human_decision_info;
    visualization_msgs::Marker robot_path_marker;
    visualization_msgs::Marker human_path_marker;

    risk_graph( int rows, int cols);
    visualization_msgs::Marker draw_human_path_in_rviz(int robot_node_number,char human_decision);
    visualization_msgs::Marker draw_robot_path_in_rviz(int robot_node_number);
    visualization_msgs::Marker delete_human_path_from_rviz();
    visualization_msgs::Marker delete_robot_path_from_rviz();
    void make_barriers();
    void print_solution();
    int min_distance();
    void dijkstra(risk_node* start);
    risk_path_cost_info  trace_back(risk_node*current,risk_node*goal);
    void local_costmap_update(int node_number);
};
//----------------------------------------------------------------------



#endif

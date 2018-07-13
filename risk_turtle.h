#ifndef RISK_TURTLE_H
#define RISK_TURTLE_H
#include "ros/ros.h"
#include "risk_graph.h"
#include "risk_map.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
class risk_robot{
  public:
    //float position[3];//x,y,z
    geometry_msgs::Pose2D pose2d;
    int node_number;
    risk_graph *graph;
    risk_map *map;
    tf::TransformBroadcaster br,br2;
    tf::Transform transform;
    tf::Transform transform2;
    char* operator_decision;
    char decision_result;
    risk_robot(ros::NodeHandle nh);
    visualization_msgs::Marker marker;
    visualization_msgs::Marker robot_marker;
    visualization_msgs::Marker human_marker;
    geometry_msgs::Twist movement;

    ros::Publisher move_pub;
    ros::Publisher marker_pub;
    ros::Publisher global_cost_pub;
    ros::Publisher local_cost_pub;

    ros::Subscriber odom_sub;
    ros::Subscriber human_command_sub;

    void send_transforms();
    void set_position(const nav_msgs::Odometry::ConstPtr& msg);
    char judge(geometry_msgs::Twist t);
    void get_human_decision(const std_msgs::String::ConstPtr& msg);
    void send_move();
    void turtle_mind();
};
//--------------------------------------------------------
#endif

#include <ros/ros.h>
#include "risk_map.h"
#include "risk_graph.h"
#include "risk_turtle.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "risk_main");
  ros::NodeHandle nh;
  ros::Rate rate(2.0);
  risk_robot turtle(nh);
  turtle.turtle_mind();
  return 0;
}

#include "risk_turtle.h"

  risk_robot::risk_robot(ros::NodeHandle nh){

    pose2d.x=0;
    pose2d.y=0;
    pose2d.theta=0;

    node_number=0;

    //define a map knowledge
    map=new risk_map;

    //get map from map server node
    if(!map->requestMap(nh))
      exit(-1);
    //set transforms origin and rotation
    transform.setOrigin   ( tf::Vector3(0,0, 0) );
    transform.setRotation ( tf::Quaternion(0, 0, 0.7071, 0.7071));
    transform2.setOrigin  ( tf::Vector3(0,0, 0.0) );
    transform2.setRotation( tf::Quaternion(0,0, 0, 1) );
    //define a graph knowledge
    graph=new risk_graph(map->rows/10,map->cols/10);

    move_pub             =   nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    marker_pub           =   nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    global_cost_pub      =   nh.advertise<nav_msgs::OccupancyGrid>("move_base/global_costmap/costmap",10);
    local_cost_pub       =   nh.advertise<nav_msgs::OccupancyGrid>("move_base/local_costmap/costmap",10);

    odom_sub             =   nh.subscribe("odom",1,&risk_robot::set_position, this);
    human_command_sub    =   nh.subscribe("chatter", 1000, &risk_robot::get_human_decision, this);

    send_transforms();
    global_cost_pub.publish(this->graph->costmap);
    graph->local_costmap_update(this->node_number);
    local_cost_pub.publish(this->graph->local_costmap);

    //publish robot path
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          exit;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
      marker=this->graph->draw_robot_path_in_rviz(this->node_number);
      marker_pub.publish(marker);

    //publish human path
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          exit;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }
      marker=this->graph->draw_human_path_in_rviz(this->node_number,'R');
      marker_pub.publish(marker);


  }
  void risk_robot::send_transforms()
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "map"));
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "world"));
  }

  char risk_robot::judge(geometry_msgs::Twist t){
    char human_decision='N';
    double cost_human_decision=100000;
    char robot_decision='N';
    double cost_robot_decision=100000;
    decision_result='N';
    int i=node_number;// robot is on i_th node of graph
    int j=0;          // future place of robot
    float weight_human=0.95;
    //int cost_threshold=1000;

    human_decision=*operator_decision;
    if (human_decision=='T')      {decision_result= human_decision; return human_decision;}
    else if(human_decision=='U')  j=i-graph->cols_l;
    else if(human_decision=='R')	j=i-1;
    else if(human_decision=='D')	j=i+graph->cols_l;
    else                          j=i+1;
    graph->dijkstra(graph->graph_nodes+i);
    graph->robot_decision_info=graph->trace_back(graph->graph_nodes+i,graph->graph_nodes+1010);   //update cost of robot decision
    //update robot decision
    graph->dijkstra(graph->graph_nodes+j);
    graph->human_decision_info=graph->trace_back(graph->graph_nodes+j,graph->graph_nodes+1010);  //update cost of human decision

    //update human decision
    std::cout<<"\nhuman_decision\t"<<human_decision<<" cost "<<graph->human_decision_info.path_cost*weight_human<<endl;
    std::cout<<"robot_decision\t"<<*(graph->robot_decision_info.path.rbegin())<<" cost "<<graph->robot_decision_info.path_cost<<endl;
    if(graph->human_decision_info.path_cost*weight_human <= graph->robot_decision_info.path_cost)
    {
      cout<<"who decided? operator\n";
      decision_result=human_decision;//*(graph->human_decision_info.path.rbegin());   //return human decision as decision_result
    }
    else
    {
      cout<<"who decided? robot\n";
      decision_result=*(graph->robot_decision_info.path.rbegin());   //return robot decision as decision_result
    }
    //bara alan
    //decision_result=human_decision;
    return decision_result;
  }
void risk_robot::set_position(const nav_msgs::Odometry::ConstPtr& msg)
{
  //Extract Position=====================================================
  pose2d.x = msg->pose.pose.position.x;
  pose2d.y = msg->pose.pose.position.y;

  tf::Quaternion q(
  msg->pose.pose.orientation.x,
  msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z,
  msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose2d.theta = yaw;
  //====================================================================
  int graph_x=0,graph_y=0;
  if((int)(pose2d.x/graph->resolution)%10>=5)           graph_y= ceil (pose2d.x/graph->resolution/10);
  else                                                  graph_y= floor(pose2d.x/graph->resolution/10);
  if((int)(pose2d.y/graph->resolution)%10>=5)           graph_x= -1 * ceil (pose2d.y/graph->resolution/10);
  else                                                  graph_x= -1 * floor(pose2d.y/graph->resolution/10);
  //node_number calculation=============================================
  int center_node=722;
  node_number= center_node + graph_y*(graph->cols_l)+graph_x;
}

void risk_robot::get_human_decision(const std_msgs::String::ConstPtr& msg)
{
  cout<<"\n______________________________________________________________________\n";

  operator_decision=strdup(msg->data.c_str());

  geometry_msgs::Twist t;
  cout<<"result = "<<this->judge(t);
  robot_marker=this->graph->draw_robot_path_in_rviz(this->node_number);
  human_marker=this->graph->draw_human_path_in_rviz(this->node_number,*operator_decision);
  this->send_move();

}

void risk_robot::send_move()
{
  float pi=3.14;
  float ep=0.1;

  movement.linear.x=0;
  movement.linear.y=0;
  movement.linear.z=0;
  movement.angular.x=0;
  movement.angular.y=0;
  movement.angular.z=0;

     if (decision_result=='T')
      {
          movement.linear.x=0.1;
          movement.linear.y=0.1;
          movement.angular.z=pi/4;
          //cout<<" --T--";
      }
    else if (decision_result=='D')
    {

        if(  pose2d.theta<ep && pose2d.theta>-1*ep)    movement.linear.x=0.3;
        else                                           movement.angular.z=-1*(pose2d.theta);// + pi/2;
        //cout<<" --D--";
    }
    else if (decision_result=='R')
    {
        if( pose2d.theta<pi/2.0+ep &&  pose2d.theta>pi/2.0-ep)      movement.linear.x=0.3;
        else                                                        movement.angular.z=-1*(pose2d.theta-pi/2.0);// + pi;
        //cout<<" --R--";
    }
    else if (decision_result=='L')
    {
        if( pose2d.theta<-1.0*pi/2.0+ep && pose2d.theta>-1.0*pi/2.0-ep)         movement.linear.x=0.3;
        else                                                                    movement.angular.z=-1*(pose2d.theta+pi/2);// + 0;
        //cout<<" --L--";
    }
    else if (decision_result=='U')
      {
        if( pose2d.theta<-1*pi+ep || pose2d.theta>pi-ep )  movement.linear.x=0.3;
        else                                               movement.angular.z=-1*(pose2d.theta-pi);// + 3*pi/2;
        //cout<<" --U--";
    }
    else
    {
       //cout<<" --N--";
    }
    //cout<<" --Moved--\n";
    move_pub.publish(movement);
}

void risk_robot::turtle_mind()
{
  geometry_msgs::Twist t;

  int numofpublishmarker=0;
  while(ros::ok())
  {
    numofpublishmarker++;
    //transforms
    this->send_transforms();
    //global cost map
    global_cost_pub.publish(this->graph->costmap);
    //local cost map
    this->graph->local_costmap_update(this->node_number);
    local_cost_pub.publish(this->graph->local_costmap);

    if(numofpublishmarker%8==1){
     while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok()) exit;
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub.publish(human_marker);
    }
    if(numofpublishmarker%8==2){
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok()) exit;
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub.publish(robot_marker);
    }
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(2));
    //cout<<"+";
  }
}

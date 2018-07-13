#include "risk_graph.h"
//llllll
#include <stdio.h>
risk_graph::risk_graph(int rows, int cols){
  num_of_barriers=12;
  barriers= new risk_barrier[12];
  make_barriers();
  cout<<"barriers set";
  costmap.info.height=rows*10;
  costmap.info.width=cols*10;
  costmap.info.resolution=0.05;
  costmap.info.origin.position.x=11.5;
  costmap.info.origin.position.y=7;
  costmap.info.origin.orientation.z=-1;
  local_costmap.info.height=rows*10;
  local_costmap.info.width=cols*10;
  local_costmap.info.resolution=0.05;
  local_costmap.info.origin.position.x=11.5;
  local_costmap.info.origin.position.y=7;
  local_costmap.info.origin.orientation.z=-1;
  resolution=0.05;
  cols_l=cols;//50
  rows_l=rows;//25
  V=rows_l*cols_l;
  graph_nodes=new risk_node[V];
  dist=new double[V];
  sptSet=new bool[V];
  for (int i=0; i<V; i++){
      ( graph_nodes + i )->x=(int)(i%cols_l)*10;
      ( graph_nodes + i )->y=(int)(i/cols_l)*10;
      ( graph_nodes + i )->edge_cost_calculation(barriers);
      printf("x%d,y%d \t edge %.1f ,%.1f ,%.1f ,%.1f \n",(i%cols_l)*10,(i/cols_l)*10,( graph_nodes + i )->edge_cost[0],( graph_nodes + i )->edge_cost[1],( graph_nodes + i )->edge_cost[2],( graph_nodes + i )->edge_cost[3]);
    }

  costmap.data.resize(rows_l*cols_l*100);
  local_costmap.data.resize(rows_l*cols_l*100);

  double cost1=0,cost2=0;
  for(int i=0;i<(rows_l)*(cols_l);i++)
    {
      cost1=((graph_nodes+i)->edge_cost[1])*10;
      cost2=((graph_nodes+i)->edge_cost[2])*10;
      for(int y_i=0;y_i<10;y_i++)
        {
          for(int x_i=0;x_i<10;x_i++)
            {
              if(x_i==0)
                costmap.data[ i/cols_l*10*cols_l*10+i%cols_l*10 + x_i+y_i*cols_l*10 ]=cost1;
              else if(y_i==0)
                costmap.data[ i/cols_l*10*cols_l*10+i%cols_l*10 + x_i+y_i*cols_l*10 ]=cost2;
              else
                costmap.data[ i/cols_l*10*cols_l*10+i%cols_l*10 + x_i+y_i*cols_l*10 ]=0;
            }
        }
    }
}
visualization_msgs::Marker risk_graph::delete_robot_path_from_rviz(){
  robot_path_marker.action = visualization_msgs::Marker::DELETE;
  return robot_path_marker;
}
visualization_msgs::Marker risk_graph::delete_human_path_from_rviz(){
  human_path_marker.action = visualization_msgs::Marker::DELETE;
  return human_path_marker;
}
visualization_msgs::Marker risk_graph::draw_robot_path_in_rviz(int robot_node_number)
{
    int goal_node=1010;
    robot_path_marker.type=visualization_msgs::Marker::LINE_STRIP;
    robot_path_marker.header.frame_id = "world";
    robot_path_marker.header.stamp = ros::Time();
    robot_path_marker.ns = "robot_lines";
    robot_path_marker.id = 0;
    robot_path_marker.action = visualization_msgs::Marker::ADD;
    robot_path_marker.pose.position.x = 0;
    robot_path_marker.pose.position.y = 0;
    robot_path_marker.pose.position.z = 0;
    robot_path_marker.pose.orientation.x = 0.0;
    robot_path_marker.pose.orientation.y = 0.0;
    robot_path_marker.pose.orientation.z = 0.0;
    robot_path_marker.pose.orientation.w = 1.0;
    robot_path_marker.scale.x = 0.05;
    robot_path_marker.scale.y = 0.05;
    robot_path_marker.scale.z = 0.05;
    robot_path_marker.color.r = 1.0f;
    robot_path_marker.color.g = 1.0f;
    robot_path_marker.color.b = 0.0f;
    robot_path_marker.color.a=1;
    robot_path_marker.lifetime = ros::Duration();

    geometry_msgs::Point p;
    char char_array[robot_decision_info.path.size()+1];
    strcpy(char_array, robot_decision_info.path.c_str());
    int center_node=722;
    float x=-((goal_node)%cols_l) *resolution*10+((center_node)%cols_l) *resolution*10;
    float y=-(goal_node)/cols_l *resolution*10+(center_node)/cols_l *resolution*10;
    p.x=x;
    p.y=y;
    p.z=0;
    robot_path_marker.points.clear();
    robot_path_marker.points.push_back(p);
    cout<< "\nRobot Action::\n";
    for(int i=0; i<robot_decision_info.path.size(); i++)
    {
        cout<<char_array[i] <<"  ";//<<x<<","<<y;
        if (char_array[i]  == 'U'){ y=y+0.5;}
        if (char_array[i]  == 'D'){ y=y-0.5;}
        if (char_array[i]  == 'L'){ x=x+0.5;}
        if (char_array[i]  == 'R'){ x=x-0.5;}
        p.x=x;
        p.y=y;
        p.z=0;
        //cout<<"  "<<x<<","<<y << "\t";
        robot_path_marker.points.push_back(p);

    }
    cout<<endl;
    return robot_path_marker;
}

visualization_msgs::Marker risk_graph::draw_human_path_in_rviz(int robot_node_number,char human_decision)
{
    int goal_node=1010;
    human_path_marker.type=visualization_msgs::Marker::LINE_STRIP;
    human_path_marker.header.frame_id = "world";
    human_path_marker.header.stamp = ros::Time();
    human_path_marker.ns = "human_lines";
    human_path_marker.id = 0;
    human_path_marker.action = visualization_msgs::Marker::ADD;
    human_path_marker.pose.position.x = 0;
    human_path_marker.pose.position.y = 0;
    human_path_marker.pose.position.z = 0;
    human_path_marker.pose.orientation.x = 0.0;
    human_path_marker.pose.orientation.y = 0.0;
    human_path_marker.pose.orientation.z = 0.0;
    human_path_marker.pose.orientation.w = 1.0;
    human_path_marker.scale.x = 0.1;
    human_path_marker.scale.y = 0.1;
    human_path_marker.scale.z = 0.1;
    human_path_marker.color.r = 1.0f;
    human_path_marker.color.g = 0.0f;
    human_path_marker.color.b = 0.0f;
    human_path_marker.color.a=1;
    human_path_marker.lifetime = ros::Duration();

    geometry_msgs::Point p;
    char char_array[human_decision_info.path.size()+2];
    //this->path.path.push_back(human_decision);
    strcpy(char_array, human_decision_info.path.c_str());
    int center_node=722;
    float x=-((goal_node)%cols_l) *resolution*10+((center_node)%cols_l) *resolution*10;
    float y=-(goal_node)/cols_l *resolution*10+(center_node)/cols_l *resolution*10;
    p.x=x;
    p.y=y;
    p.z=0;

    human_path_marker.points.clear();
    human_path_marker.points.push_back(p);
    cout<< "Human Action::\n";
    for(int i=0; i<human_decision_info.path.size(); i++)
    {
        cout<<char_array[i] <<"  ";//<<x<<","<<y ;
        if (char_array[i]  == 'U'){ y=y+0.5;}
        if (char_array[i]  == 'D'){ y=y-0.5;}
        if (char_array[i]  == 'L'){ x=x+0.5;}
        if (char_array[i]  == 'R'){ x=x-0.5;}
        p.x=x;
        p.y=y;
        p.z=0;
        //cout<<"  "<<x<<","<<y << "\t";
        human_path_marker.points.push_back(p);
    }
    cout<<endl;
    return human_path_marker;
}

void risk_graph::make_barriers(){
    barriers[0].x=500-419;		barriers[0].y=252-132;		barriers[0].r=20;
    barriers[1].x=500-399;		barriers[1].y=252-132;    barriers[1].r=20;
    barriers[2].x=500-379;		barriers[2].y=252-136;		barriers[2].r=20;
    barriers[3].x=500-359;    barriers[3].y=252-142;		barriers[3].r=20;
    barriers[4].x=500-341;		barriers[4].y=252-131;		barriers[4].r=20;
    barriers[5].x=190;      	barriers[5].y=252-60;     barriers[5].r=13;
    barriers[6].x=282;        barriers[6].y=252-65;     barriers[6].r=16;
    barriers[7].x=376;        barriers[7].y=252-65;     barriers[7].r=16;
    barriers[8].x=337;        barriers[8].y=140;        barriers[8].r=16;
    barriers[9].x=500-283;		barriers[9].y=60;         barriers[9].r=16;
    barriers[10].x=500-222;   barriers[10].y=77;        barriers[10].r=16;
    barriers[11].x=500-123;   barriers[11].y=78;        barriers[11].r=16;
  }


void risk_graph::print_solution()
{
   //printf("Vertex \t Distance from Source\n");
   for (int i = 0; i < cols_l; i++)
     for (int j = 0; j < rows_l; j++)
       if(dist[i*cols_l+j]!=INT_MAX)ROS_INFO("\n node: %d,%d \n cost: %f \n parent: %c\n", (graph_nodes + i*cols_l+j)->x,(graph_nodes + i*cols_l+j)->y, dist[i*cols_l+j],(graph_nodes + i*cols_l+j )->parent);
}
int risk_graph::min_distance()
{
   double min = INT_MAX;
   int min_index;

   for (int v = 0; v < V; v++)
   {if (sptSet[v] == 0 && dist[v] <= min)
         {
       min = dist[v];
       min_index = v;
         }
    //if(sptSet[v]!=0)printf("(%d,%.2f)\n",v,dist[v]);
   }
   return min_index;
}

void risk_graph::dijkstra(risk_node* start){
  bool adj_condition_1=0,adj_condition_2=0,adj_condition_3=0,adj_condition_4=0,adj_condition=0;
  double weight=0;
  for (int i = 0; i < V; i++)
          dist[i] = INT_MAX, sptSet[i] = false;
  dist[ (start->x)/10 + (start->y)/10*cols_l] = 0;
  // Find shortest path for all vertices
  for(int count=0;count<V-1;count++)
    {
      int u = min_distance();
      sptSet[u] = true;
      int u_x=u%cols_l*10;
      int u_y=u/cols_l*10;
      //printf("\ncnt %d   \t x %d   \t y %d   \t u %d   \t dist %.1f   \t actions ",count,u_x,u_y ,u,dist[u]);
      int v=0;
      //u down
      if(u_y!=0){
        //printf(" CD ");
        v= u-cols_l;//u_x*rows + u_y-1;
        weight=(graph_nodes+u)->edge_cost[2];//URDL 0123
        if (!sptSet[v]  && dist[u]!= INT_MAX && dist[u]+weight < dist[v])
        {
          dist[v] = dist[u] + weight;
          (graph_nodes+v)->parent='D';
          //printf("D %.2f ",dist[v]);
        }
      }
      //u up
      if(u_y!=(rows_l-1)*10){
        //printf(" CU ");
        v= u+cols_l;//u_x*rows + u_y+1;
        weight=(graph_nodes+u)->edge_cost[0];//URDL 0123
        if (!sptSet[v]  && dist[u]!= INT_MAX && dist[u]+weight < dist[v])
        {
          dist[v] = dist[u] + weight;
          (graph_nodes+v)->parent='U';
          //printf("U %.2f ",dist[v]);
        }
      }
      //u left
      if(u_x!=(cols_l-1)*10){
        //printf(" CL ");
        v= u+1;//(u_x+1)*rows + u_y;
        weight=(graph_nodes+u)->edge_cost[3];//URDL 0123
        if (!sptSet[v]  && dist[u]!= INT_MAX && dist[u]+weight < dist[v])
        {
          dist[v] = dist[u] + weight;
          (graph_nodes+v)->parent='L';
          //printf("L %.2f ",dist[v]);
        }
      }
      //u right
      if(u_x!=0){
       //printf(" CR ");
        v= u-1;//(u_x-1)*rows + u_y;
        weight=(graph_nodes+u)->edge_cost[1];//URDL 0123
        if (!sptSet[v]  && dist[u]!= INT_MAX && dist[u]+weight < dist[v])
        {
          dist[v] = dist[u] + weight;
          (graph_nodes+v)->parent='R';
          //printf("R %.2f ",dist[v]);
        }
      }
    }

  //for(int i=0;i<V;i++)
    //if(dist[i]==INT_MAX && sptSet[i]==false)printf("\nINT_MAX %d",i);
}
risk_path_cost_info  risk_graph::trace_back(risk_node*current,risk_node*goal)
{
  risk_node* trace_node=goal;
  path.max_path_cost=0;
  path.path_cost=0;
  path.path.clear();
  //double max_edge_cost=-1000;
  //printf("\npath:from (%d,%d) parent:%c  ",goal->x,goal->y,goal->parent);
  while(trace_node!=current)
  {
    if(trace_node->parent=='U')
    {
      //printf("U");
      path.path.push_back('U');
      if (trace_node->edge_cost[0]>path.max_path_cost)   path.max_path_cost=trace_node->edge_cost[0];
      path.path_cost+=trace_node->edge_cost[0];
      trace_node=trace_node-cols_l;
    }
    else if(trace_node->parent=='R')
    {
      //printf("R");
      path.path.push_back('R');
      if (trace_node->edge_cost[1]>path.max_path_cost)   path.max_path_cost=trace_node->edge_cost[1];
      path.path_cost+=trace_node->edge_cost[1];
      trace_node=trace_node+1;
    }
    else if(trace_node->parent=='D')
    {
      //printf("D");
      path.path.push_back('D');
      if (trace_node->edge_cost[2]>path.max_path_cost)   path.max_path_cost=trace_node->edge_cost[2];
      path.path_cost+=trace_node->edge_cost[2];
      trace_node=trace_node+cols_l;
    }
    else if(trace_node->parent=='L')
    {
      //printf("L");
      path.path.push_back('L');
      if (trace_node->edge_cost[3]>path.max_path_cost)   path.max_path_cost=trace_node->edge_cost[3];
      path.path_cost+=trace_node->edge_cost[3];
      trace_node=trace_node-1;
    }
    //printf(" (x%d,y%d) ",trace_node->x,trace_node->y);
  }
  //printf("\nmax_edge_cost:  %.2f",path.max_path_cost);
  return path;
}

void risk_graph::local_costmap_update(int node_number)//const robot& turtlebot
{
  for(int i=0;i<(rows_l)*(cols_l)*100;i++)
      local_costmap.data[i]=0;
  double local_cost_value=((graph_nodes+node_number)->edge_cost[1])+((graph_nodes+node_number)->edge_cost[2])+((graph_nodes+node_number+rows_l+1)->edge_cost[0])+((graph_nodes+node_number+rows_l+1)->edge_cost[3]);
  local_cost_value=(int)(10*local_cost_value/4);
  for(int x_i=0;x_i<10;x_i++)
    for(int y_i=0;y_i<10;y_i++)
      local_costmap.data[node_number/cols_l*10*cols_l*10+node_number%cols_l*10 + x_i+y_i*cols_l*10]=local_cost_value;
}
//---------------------------------------------------------------------------

risk_node::risk_node(){
  parent='N';//null parent @ initialization
}

void risk_node::edge_cost_calculation(risk_barrier barriers[12]){
  int num_of_barriers=12;//num_of_barrierser of barriers
  //find nearest barrier
  double distance=0,min_distance=10000000000,distance1=0,distance2=0;
  int x2=0,y2=0,index_0=0,index_1=0,index_2=0,index_3=0;
  //up
        x2=x, y2=y+1;
        for(int i=0;i<num_of_barriers;i++){
          distance1=sqrt(pow(x-barriers[i].x,2)+pow(y-barriers[i].y,2))-barriers[i].r;
          distance2=sqrt(pow(x2-barriers[i].x,2)+pow(y2-barriers[i].y,2))-barriers[i].r;
          distance=distance1<distance2?distance1:distance2;
          if( distance < min_distance )
            {min_distance=distance;
             index_0=i;
            }
        }
        if (min_distance<3)	edge_cost[0]=255;
        else	edge_cost[0]=255/double(min_distance);
  //right
        x2=x+1,y2=y;
        distance=0,min_distance=10000000000,distance1=0,distance2=0;
        for(int i=0;i<num_of_barriers;i++){
          distance1=sqrt(pow(x-barriers[i].x,2)+pow(y-barriers[i].y,2));
          distance2=sqrt(pow(x2-barriers[i].x,2)+pow(y2-barriers[i].y,2));
          distance=distance1<distance2?distance1:distance2;
          if( distance < min_distance )
            {min_distance=distance;
            index_1=i;}
        }
        if (min_distance<3)	edge_cost[1]=255;
        else	edge_cost[1]=255/double(min_distance);
  //down
        x2=x, y2=y-1;
        distance=0,min_distance=10000000000,distance1=0,distance2=0;
        for(int i=0;i<num_of_barriers;i++){
          distance1=sqrt(pow(x-barriers[i].x,2)+pow(y-barriers[i].y,2));
          distance2=sqrt(pow(x2-barriers[i].x,2)+pow(y2-barriers[i].y,2));
          distance=distance1<distance2?distance1:distance2;
          if( distance < min_distance )
            {min_distance=distance;
            index_2=i;
            }
        }
        if (min_distance<3)	edge_cost[2]=255;
        else	edge_cost[2]=255/double(min_distance);
  //left
        x2=x-1,y2=y;
        distance=0,min_distance=10000000000,distance1=0,distance2=0;
        for(int i=0;i<num_of_barriers;i++){
          distance1=sqrt(pow(x-barriers[i].x,2)+pow(y-barriers[i].y,2));
          distance2=sqrt(pow(x2-barriers[i].x,2)+pow(y2-barriers[i].y,2));
          distance=distance1<distance2?distance1:distance2;
          if( distance < min_distance )
            {min_distance=distance;
            index_3=i;
            }
        }
        if (min_distance<3)	edge_cost[3]=255;
        else	edge_cost[3]=255/double(min_distance);
  //ROS_INFO("pose: (%d,%d)\t cost: (%f,%d,%f,%d,%f,%d,%f,%d)\n ",x,y,edge_cost[0],index_0,edge_cost[1],index_1,edge_cost[2],index_2,edge_cost[3],index_3);
}
//-----------------------------------------------------------------------------------

risk_path_cost_info::risk_path_cost_info()
{
  path_cost=0;
  max_path_cost=0;
}

//-----------------------------------------------------------------------------------















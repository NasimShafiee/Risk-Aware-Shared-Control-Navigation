#include "risk_map.h"

void risk_map::readMap(const nav_msgs::OccupancyGrid& map)
{
    rows = map.info.height;
    cols = map.info.width;
    mapResolution = map.info.resolution;
    my_map.info=map.info;
    my_map.info.width=map.info.width;
    my_map.info.height=map.info.height;
    my_map.info.resolution=map.info.resolution;
    my_map.data.resize(map.info.width*map.info.height);
    for(int i=0;i<map.info.width*map.info.height;i++)
      my_map.data[i]=map.data[i];
    // Dynamically resize the grid
    grid.resize(rows);
    for (int i = 0; i < rows; i++) {
        grid[i].resize(cols);
    }
    int currCell = 0;
    for (int i = 0; i < rows; i++)  {
        for (int j = 0; j < cols; j++)      {
            if (map.data[currCell] == 0) // unoccupied cell
                grid[i][j] = false;
            else
                grid[i][j] = true; // occupied (100) or unknown cell (-1)
            currCell++;
        }
    }
}


bool risk_map::requestMap(ros::NodeHandle &nh)
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;
    while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
    {
         ROS_INFO("Waiting for service static_map to become available");
    }
    ROS_INFO("Requesting the map...");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
    if (mapClient.call(req, res))
    {
        readMap(res.map);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

void risk_map::printGrid()
{
    printf("Grid map:\n");
    int freeCells = 0;
    for (int i = 0; i < rows; i++)
    {
        printf("\nRow no. %d\n", i);
        for (int j = 0; j < cols; j++)
            printf("%d ", grid[i][j] ? 1 : 0);
    }
}

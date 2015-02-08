#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/thread.hpp>
#include <math.h>

ros::Subscriber map_sub;
ros::Publisher obstacle_publisher;

void map_cb(const nav_msgs::OccupancyGridConstPtr &map)
{
  nav_msgs::GridCells obstacles;
  obstacles.header.stamp = map->header.stamp;
  obstacles.header.frame_id = map->header.frame_id;
  obstacles.cell_width = map->info.resolution;
  obstacles.cell_height = map->info.resolution;

  for (unsigned int i = 0 ; i < map->info.height; ++i)
  {
    for(unsigned int j = 0; j < map->info.width; ++j)
    {
      if(map->data[i*map->info.height+j] == 100)
      {
        geometry_msgs::Point obstacle_coordinates;
        obstacle_coordinates.x = (j * obstacles.cell_height) + map->info.origin.position.x + (map->info.resolution/2.0);
        obstacle_coordinates.y = (i * obstacles.cell_width) + map->info.origin.position.y + (map->info.resolution/2.0);
        obstacle_coordinates.z = 0;
        obstacles.cells.push_back(obstacle_coordinates);
      }
    }
  }

  obstacle_publisher.publish(obstacles);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "costmap_translator");
  ros::NodeHandle nh;

  obstacle_publisher = nh.advertise<nav_msgs::GridCells>("/costmap_translator/obstacles", 1);
  map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/costmap_server/costmap/costmap", 1, map_cb);

  ros::spin();
}

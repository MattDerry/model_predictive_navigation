#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
namespace costmap_2d
{
  class Costmap2DNode
  {
    public:
      Costmap2DNode(tf::TransformListener& tf) : costmap_ros_("costmap", tf) {}
    private:
      Costmap2DROS costmap_ros_;
  };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_node");

  tf::TransformListener tf(ros::Duration(10));

  costmap_2d::Costmap2DNode* costmap_node;
  costmap_node = new costmap_2d::Costmap2DNode(tf);

  ros::spin();

  delete costmap_node;

  return(0);
}

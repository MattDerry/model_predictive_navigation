#include <model_predictive_navigation/control_law.h>
#include <model_predictive_navigation/EgoGoal.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/Joy.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <tf/transform_datatypes.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include <vector>
#include <string>

using ::model_predictive_navigation::ControlLaw;
using ::model_predictive_navigation::ControlLawSettings;
using ::model_predictive_navigation::EgoPolar;
using ::model_predictive_navigation::EgoGoal;

#define RATE_FACTOR 0.5
#define DEFAULT_LOOP_RATE 25

#define RESULT_BEGIN 1
#define RESULT_SUCCESS 2
#define RESULT_CANCEL 3

// Trajectory model parameters
#define K_1 1.2           // 2
#define K_2 3             // 8
#define BETA 0.4          // 0.5
#define LAMBDA 2          // 3
#define R_THRESH 0.15
#define V_MAX 0.3         // 0.3
#define V_MIN 0.00

#define R_SPEED_LIMIT 0.5
#define V_SPEED_LIMIT 0.3

// Parameters to determine success
#define GOAL_DIST_UPDATE_THRESH   0.15   // in meters
#define GOAL_ANGLE_UPDATE_THRESH  0.1   // in radians

#define GOAL_DIST_ID_THRESH   0.1       // in meters
#define GOAL_ANGLE_ID_THRESH  0.3       // in radians

static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;
static const double minusPI= -3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

ros::Publisher cmd_pub, current_goal_pub, nav_result_pub, goal_request_pub, rviz_goal_request_pub, min_Obstacle_pub;
ros::Subscriber goal_sub, odom_sub, joy_sub, cost_map_sub, int_goal_pose_sub;

ros::ServiceServer cancel_goal_service;
ros::NodeHandle * nh;

geometry_msgs::PoseStamped global_goal_pose, inter_goal_pose;
EgoPolar inter_goal_coords;
double inter_goal_vMax, inter_goal_k1, inter_goal_k2;
nav_msgs::Odometry current_pose;
std::vector<geometry_msgs::PoseStamped> plan;
nav_msgs::GridCells cost_map;

std_msgs::Int32 result;
bool bFollowingTrajectory, bHasGoal, bHasIntGoal, prevButton, bHasCostMap, bHasOdom;
boost::mutex pose_mutex_, cost_map_mutex_, int_goal_mutex_;

ControlLaw * cl;

boost::mutex goals_mutex_;
std::vector<geometry_msgs::PoseStamped> current_goals;
unsigned int current_nav_goal_idx;
bool bCurrentStepIsTrajectoryEnd;

double distance(double pose_x, double pose_y, double obx, double oby)
{
  double diffx = obx - pose_x;
  double diffy = oby - pose_y;
  double dist = sqrt(diffx*diffx + diffy*diffy);
  return dist;
}

void cancel_auton_nav(int res)
{
  bFollowingTrajectory = false;
  bHasIntGoal = false;
  result.data = res;
  nav_result_pub.publish(result);
}

void int_goal_cb(const model_predictive_navigation::EgoGoal::ConstPtr& ego_goal)
{
  boost::mutex::scoped_lock lock(pose_mutex_);

  inter_goal_coords.r = ego_goal->r;
  inter_goal_coords.delta = ego_goal->delta;
  inter_goal_coords.theta = ego_goal->theta;
  inter_goal_vMax = ego_goal->vMax;
  inter_goal_k1 = ego_goal->k1;
  inter_goal_k2 = ego_goal->k2;

  inter_goal_pose.header.stamp = ros::Time::now();
  inter_goal_pose.header.frame_id = "/odom";
  inter_goal_pose.pose = cl->convert_from_egopolar(current_pose, inter_goal_coords);

  bHasIntGoal = true;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
  boost::mutex::scoped_lock lock(pose_mutex_);
  current_pose = *pose;
  bHasOdom = true;
}

void nav_cb(const nav_msgs::GridCells::ConstPtr& grid_cells)
{
  boost::mutex::scoped_lock lock(cost_map_mutex_);
  cost_map = *grid_cells;
  bHasCostMap = true;
}

float goal_dist(geometry_msgs::PoseStamped gp)
{
  return sqrt(((current_goals[0].pose.position.x - gp.pose.position.x)
             * (current_goals[0].pose.position.x - gp.pose.position.x)) +
              ((current_goals[0].pose.position.y - gp.pose.position.y)
             * (current_goals[0].pose.position.y - gp.pose.position.y)));
}

float goal_angle_dist(geometry_msgs::PoseStamped gp)
{
  return fabs(tf::getYaw(current_goals[0].pose.orientation) - tf::getYaw(gp.pose.orientation));
}

bool same_global_goal(geometry_msgs::PoseStamped new_goal)
{
  bool isSameGoal = false;

  if (bHasGoal)
  {
    if (goal_dist(new_goal) < GOAL_DIST_ID_THRESH)
    {
      if (goal_angle_dist(new_goal) < GOAL_ANGLE_ID_THRESH)
      {
        isSameGoal = true;
      }
    }
  }

  return isSameGoal;
}

void receive_goals(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  boost::mutex::scoped_lock lock(goals_mutex_);

  bool bIsSameGoal = same_global_goal(*goal);

  current_goals.clear();
  current_goals.push_back(*goal);

  if (bHasOdom && bHasCostMap)
  {
    global_goal_pose = *goal;
  }
  else
  {
    ROS_DEBUG("[MPEPC] goal recieved but no odom or cost map");
    return;
  }

  if (bFollowingTrajectory && !bIsSameGoal)
  {
    ROS_DEBUG("cancelling current goal");
    cancel_auton_nav(RESULT_CANCEL);
  }

  bHasGoal = true;

  if (!bFollowingTrajectory && bHasGoal)
  {
    ROS_INFO("Begin path planning and execution");
    rviz_goal_request_pub.publish(global_goal_pose);
    bFollowingTrajectory = true;
    result.data = RESULT_BEGIN;
    goal_request_pub.publish(global_goal_pose);
    nav_result_pub.publish(result);
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "mpepc_trajectory");
  ros::NodeHandle n;
  nh = &n;

  // Setup control law for trajectory generation
  ControlLawSettings settings;
  settings.m_K1 = K_1;
  settings.m_K2 = K_2;
  settings.m_BETA = BETA;
  settings.m_LAMBDA = LAMBDA;
  settings.m_R_THRESH = R_THRESH;
  settings.m_V_MAX = V_MAX;
  settings.m_V_MIN = V_MIN;
  ControlLaw claw(settings);
  cl = &claw;

  current_pose.pose.pose.orientation.x = 0;
  current_pose.pose.pose.orientation.y = 0;
  current_pose.pose.pose.orientation.z = 0;
  current_pose.pose.pose.orientation.w = 1;

  bFollowingTrajectory = false;
  bHasGoal = false;
  bHasIntGoal = false;
  prevButton = false;
  bHasOdom = false;
  bHasCostMap = true;

  goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/mpepc_goal_request", 1, receive_goals);
  cmd_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  nav_result_pub = nh->advertise<std_msgs::Int32>("nav_result", 1);
  goal_request_pub = nh->advertise<geometry_msgs::PoseStamped>("goal_request", 1);
  rviz_goal_request_pub = nh->advertise<geometry_msgs::PoseStamped>("rviz_goal_request", 1);

  odom_sub = nh->subscribe<nav_msgs::Odometry>("/odom", 1, odom_cb);
  int_goal_pose_sub = nh->subscribe<model_predictive_navigation::EgoGoal>("int_goal_pose", 1, int_goal_cb);
  cost_map_sub = nh->subscribe<nav_msgs::GridCells>("/costmap_translator/obstacles", 100, nav_cb);

  // ros::Rate loop_rate(20.0);
  double loop_rate = DEFAULT_LOOP_RATE;
  double global_loop_rate;
  if (ros::param::has("/global_loop_rate"))
  {
    nh->getParam("/global_loop_rate", global_loop_rate);
    loop_rate= RATE_FACTOR*global_loop_rate;
    ROS_DEBUG("Smooth_Traj: Loop rate updated using global_loop_rate to : %f", loop_rate);
  }
  ros::Rate rate_obj(loop_rate);

  EgoPolar global_goal_coords;
  geometry_msgs::Twist cmd_vel;

  // Control Loop and code
  while (ros::ok())
  {
    if (bFollowingTrajectory && bHasIntGoal)  // autonomous trajectory generation and following mode
    {
      global_goal_coords = cl->convert_to_egopolar(current_pose, global_goal_pose.pose);
      if (global_goal_coords.r <= GOAL_DIST_UPDATE_THRESH)
      {
        double angle_error = tf::getYaw(current_pose.pose.pose.orientation) - tf::getYaw(global_goal_pose.pose.orientation);
        angle_error = cl->wrap_pos_neg_pi(angle_error);
        if (fabs(angle_error) > GOAL_ANGLE_UPDATE_THRESH)
        {
          cmd_vel.linear.x = 0;
          if (angle_error > 0)
            cmd_vel.angular.z = -4 * settings.m_V_MIN;
          else
            cmd_vel.angular.z = 4 * settings.m_V_MIN;
        }
        else
        {
          ROS_DEBUG("[MPEPC] Completed normal trajectory following");
          cancel_auton_nav(RESULT_SUCCESS);
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
        }
      }
      else
      {
        cmd_vel = cl->get_velocity_command(current_pose, inter_goal_pose.pose, inter_goal_k1, inter_goal_k2, inter_goal_vMax);
      }
    }
    cmd_pub.publish(cmd_vel);

    ros::spinOnce();
    rate_obj.sleep();
  }
}

#include <model_predictive_navigation/control_law.h>
#include <model_predictive_navigation/EgoGoal.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>

#include <math.h>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nlopt.hpp>
#include "flann/flann.hpp"

using ::model_predictive_navigation::ControlLaw;
using ::model_predictive_navigation::ControlLawSettings;
using ::model_predictive_navigation::EgoPolar;
using ::model_predictive_navigation::EgoGoal;

#define RATE_FACTOR 0.2
#define DEFAULT_LOOP_RATE 10

#define RESULT_BEGIN 1
#define RESULT_SUCCESS 2
#define RESULT_CANCEL 3

// Trajectory Model Params
#define K_1 1.2           // 2
#define K_2 3             // 8
#define BETA 0.4          // 0.5
#define LAMBDA 2          // 3
#define R_THRESH 0.05
#define V_MAX 0.3         // 0.3
#define V_MIN 0.0

// Trajectory Optimization Params
#define TIME_HORIZON 5.0
#define DELTA_SIM_TIME 0.2
#define SAFETY_ZONE 0.125
#define WAYPOINT_THRESH 1.75

// Cost function params
static const double C1 = 0.05;
static const double C2 = 2.5;
static const double C3 = 0.05;        // 0.15
static const double C4 = 0.05;        // 0.2 //turn
static const double PHI_COL = 1.0;   // 0.4
static const double SIGMA = 0.2;    // 0.10

static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;
static const double minusPI= -3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

ros::NodeHandle * nh;

boost::mutex goals_mutex_;
vector<geometry_msgs::PoseStamped> current_goals;

ros::Publisher ego_goal_pub, plan_pub, min_goal_obstacle_pub, traj_pub;
ros::Subscriber goal_pose_sub, odom_sub, nav_result_sub, cost_map_sub;

geometry_msgs::PoseStamped global_goal_pose, current_pose_stamp;
nav_msgs::Odometry current_pose;
nav_msgs::GridCells cost_map;

std_msgs::Int32 current_status;
bool bHasGoal, bHasCostMap, bHasOdom;
bool bUseWaypointFollowing;
boost::mutex pose_mutex_, cost_map_mutex_;

navfn::NavfnROS nav_fn;
costmap_2d::Costmap2DROS * costmap;

ControlLaw * cl;

flann::Index<flann::L2<float> > * obs_tree;
flann::Matrix<float> * data;

int trajectory_count;

double map_resolution;
double interp_rotation_factor;

struct Point {
  float a;
  float b;
  int member;
  int p_idx;
  Point(float x, float y) : a(x), b(y), member(-1), p_idx(0) {}
  Point() : a(0), b(0), member(-1), p_idx(0) {}
  inline bool operator==(Point p) {
     if (p.a == a && p.b == b)
        return true;
     else
        return false;
  }
};

struct MinDistResult {
  Point p;
  double dist;
};

double mod(double x, double y)
{
  double m= x - y * floor(x/y);
  // handle boundary cases resulted from floating-point cut off:
  if (y > 0)              // modulo range: [0..y)
  {
    if (m >= y)           // Mod(-1e-16             , 360.    ): m= 360.
      return 0;

    if (m < 0)
    {
      if (y+m == y)
        return 0;     // just in case...
      else
        return y+m;  // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
    }
  }
  else                    // modulo range: (y..0]
  {
    if (m <= y)           // Mod(1e-16              , -360.   ): m= -360.
      return 0;

    if (m>0 )
    {
      if (y+m == y)
        return 0;    // just in case...
      else
        return y+m;  // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
    }
  }

  return m;
}

vector<MinDistResult> find_points_within_threshold(Point newPoint, double threshold)
{
  vector<MinDistResult> results;

  flann::Matrix<float> query(new float[2], 1, 2);
  query[0][0] = newPoint.a;
  query[0][1] = newPoint.b;

  std::vector< std::vector<int> > indices;
  std::vector< std::vector<float> > dists;

  flann::SearchParams params;
  params.checks = 128;
  params.max_neighbors = -1;
  params.sorted = true;
  // ROS_INFO("Do search");
  {
    boost::mutex::scoped_lock lock(cost_map_mutex_);
    obs_tree->radiusSearch(query, indices, dists, threshold, params);

    // ROS_INFO("Finished search");
    for (int i = 0; i < indices[0].size(); i++)
    {
      MinDistResult result;
      result.p = Point((*data)[indices[0][i]][0], (*data)[indices[0][i]][1]);
      result.dist = static_cast<double>(dists[0][i]);
      results.push_back(result);
    }
  }

  delete[] query.ptr();
  indices.clear();
  dists.clear();

  return results;
}

MinDistResult find_nearest_neighbor(Point queryPoint)
{
  MinDistResult results;

  flann::Matrix<float> query(new float[2], 1, 2);
  query[0][0] = queryPoint.a;
  query[0][1] = queryPoint.b;

  std::vector< std::vector<int> > indices;
  std::vector< std::vector<float> > dists;

  flann::SearchParams params;
  params.checks = 128;
  params.sorted = true;

  {
    boost::mutex::scoped_lock lock(cost_map_mutex_);
    obs_tree->knnSearch(query, indices, dists, 1, params);
    results.p = Point((*data)[indices[0][0]][0], (*data)[indices[0][0]][1]);
    results.dist = static_cast<double>(dists[0][0]);
  }

  MinDistResult tempResults;
  tempResults.p = Point(cost_map.cells[indices[0][0]].x, cost_map.cells[indices[0][0]].y);

  delete[] query.ptr();
  indices.clear();
  dists.clear();

  return results;
}

double distance(double pose_x, double pose_y, double obx, double oby)
{
  double diffx = obx - pose_x;
  double diffy = oby - pose_y;
  double dist = sqrt(diffx*diffx + diffy*diffy);
  return dist;
}

double min_distance_to_obstacle(geometry_msgs::Pose local_current_pose, double *heading)
{
  // ROS_INFO("In minDist Function");
  Point global(local_current_pose.position.x, local_current_pose.position.y);
  MinDistResult nn_graph_point = find_nearest_neighbor(global);

  double minDist = 100000;
  double head = 0;

  double SOME_THRESH = 1.5;

  if(nn_graph_point.dist < SOME_THRESH)
  {
    int min_i = 0;
    vector<MinDistResult> distResult;
    distResult = find_points_within_threshold(global, 1.1*SOME_THRESH);

    //ROS_INFO("Loop through %d points from radius search", distResult.size());
    for (unsigned int i = 0 ; i < distResult.size() && minDist > 0; i++)
    {
      double dist = distance(local_current_pose.position.x, local_current_pose.position.y, cost_map.cells[i].x, cost_map.cells[i].y);
      if (dist < minDist)
      {
        minDist = dist;
        min_i = i;
      }
    }

    // ROS_INFO("Calculate heading");
    head = tf::getYaw(local_current_pose.orientation) - atan2(cost_map.cells[min_i].y - local_current_pose.position.y, cost_map.cells[min_i].x - local_current_pose.position.x);
    head = mod(head + PI, TWO_PI) - PI;
    //ROS_INFO("Got nearest radius neighbor, poly dist: %f", minDist);
  }
  else
  {
    minDist = distance(local_current_pose.position.x, local_current_pose.position.y, nn_graph_point.p.a, nn_graph_point.p.b);
    //ROS_INFO("Got nearest neighbor, poly dist: %f", minDist);
  }

  *heading = head;

  return minDist;
}

float get_obstacle_distance()
{
  if (!bHasCostMap || !bHasOdom)
  {
    return 0;
  }

  double heading = 0.0;

  nav_msgs::Odometry cur_pose;
  {
    boost::mutex::scoped_lock lock(pose_mutex_);
    cur_pose = current_pose;
  }

  return min_distance_to_obstacle(cur_pose.pose.pose, &heading);
}

float get_goal_distance()
{
  if (!bHasCostMap || !bHasOdom)
  {
    return 0;
  }

  nav_msgs::Odometry cur_pose;
  {
    boost::mutex::scoped_lock lock(pose_mutex_);
    cur_pose = current_pose;
  }

  return distance(global_goal_pose.pose.position.x, global_goal_pose.pose.position.y, cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y);
}

// Bilinear Interpolation on navigation function
double get_interpolated_point_potential(geometry_msgs::Point position)
{
  costmap_2d::Costmap2D * temp_costmap;
  temp_costmap = costmap->getCostmap();

  // find world coords of current cell
  double cell_wx, cell_wy;
  unsigned int cell_mx, cell_my;
  temp_costmap->worldToMap(position.x, position.y, cell_mx, cell_my);
  temp_costmap->mapToWorld(cell_mx, cell_my, cell_wx, cell_wy);

  geometry_msgs::Point tempPoint = position;

  tempPoint.x = tempPoint.x + map_resolution;
  double cost0 = nav_fn.getPointPotential(tempPoint);

  tempPoint = position;
  tempPoint.y = tempPoint.y + map_resolution;
  double cost90 = nav_fn.getPointPotential(tempPoint);

  tempPoint = position;
  tempPoint.x = tempPoint.x - map_resolution;
  double cost180 = nav_fn.getPointPotential(tempPoint);

  // Block at 270
  tempPoint = position;
  tempPoint.y = tempPoint.y - map_resolution;
  double cost270 = nav_fn.getPointPotential(tempPoint);

  geometry_msgs::Point rotPoint;
  rotPoint.x = ((position.x - cell_wx)*cos(-1*PI/4) - (position.y - cell_wy)*sin(-1*PI/4)) + interp_rotation_factor;
  rotPoint.y = ((position.x - cell_wx)*sin(-1*PI/4) + (position.y - cell_wy)*cos(-1*PI/4)) + interp_rotation_factor;

  double a00 = cost180;
  double a10 = cost270 - cost180;
  double a01 = cost90 - cost180;
  double a11 = cost180 - cost270 - cost90 + cost0;

  return a00 + a10*rotPoint.x + a01*rotPoint.y + a11*rotPoint.x*rotPoint.y;
}

// This function is used by the optimizer to score different trajectories
double sim_trajectory(double r, double delta, double theta, double vMax, double time_horizon)
{
  nav_msgs::Odometry sim_pose;

  {
    boost::mutex::scoped_lock lock(pose_mutex_);
    sim_pose = current_pose;
  }

  EgoPolar sim_goal;
  sim_goal.r = r;
  sim_goal.delta = delta;
  sim_goal.theta = theta;

  geometry_msgs::Pose current_goal = cl->convert_from_egopolar(sim_pose, sim_goal);

  double SIGMA_DENOM = pow(SIGMA, 2);

  double sim_clock = 0.0;

  geometry_msgs::Twist sim_cmd_vel;
  double current_yaw = tf::getYaw(sim_pose.pose.pose.orientation);
  geometry_msgs::Point collisionPoint;
  bool collision_detected = false;

  double expected_progress = 0.0;
  double expected_action = 0.0;
  double expected_collision = 0.0;

  double nav_fn_t0 = 0;
  double nav_fn_t1 = 0;
  double collision_prob = 0.0;
  double survivability = 1.0;
  double obstacle_heading = 0.0;

  while (sim_clock < time_horizon)
  {
    // Get Velocity Commands
    sim_cmd_vel = cl->get_velocity_command(sim_goal, vMax);

    // get navigation function at orig pose
    nav_fn_t0 = get_interpolated_point_potential(sim_pose.pose.pose.position);

    // Update pose
    current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
    sim_pose.pose.pose.position.x = sim_pose.pose.pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
    sim_pose.pose.pose.position.y = sim_pose.pose.pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
    sim_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);

    // Get navigation function at new pose
    nav_fn_t1 = get_interpolated_point_potential(sim_pose.pose.pose.position);

    double minDist = min_distance_to_obstacle(sim_pose.pose.pose, &obstacle_heading);

    if (minDist <= SAFETY_ZONE)
    {
      // ROS_INFO("Collision Detected");
      collision_detected = true;
    }

    // Get collision probability
    if (!collision_detected)
    {
      collision_prob = exp(-1*pow(minDist, 2)/SIGMA_DENOM);  // sigma^2
    }
    else
    {
      collision_prob = 1;
    }

    // Get survivability
    survivability = survivability*(1 - collision_prob);

    expected_collision = expected_collision + ((1-survivability) * C2);

    // Get progress cost
    expected_progress = expected_progress + (survivability * (nav_fn_t1 - nav_fn_t0));

    // Get action cost
    expected_action = expected_action + (C3 * pow(sim_cmd_vel.linear.x, 2) + C4 * pow(sim_cmd_vel.angular.z, 2))*DELTA_SIM_TIME;

    // Calculate new EgoPolar coords for goal
    sim_goal = cl->convert_to_egopolar(sim_pose, current_goal);

    sim_clock = sim_clock + DELTA_SIM_TIME;
  }

  // Update with angle heuristic - weighted difference between final pose and gradient of navigation function
  double gradient_angle = get_interpolated_point_potential(sim_pose.pose.pose.position);

  expected_progress = expected_progress + C1 * abs(tf::getYaw(sim_pose.pose.pose.orientation) - gradient_angle);

  ++trajectory_count;

  // SUM collision cost, progress cost, action cost
  return (expected_collision + expected_progress + expected_action);
}

double score_trajectory(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
  double time_horizon = TIME_HORIZON;
  return sim_trajectory(x[0], x[1], x[2], x[3], time_horizon);
}

// Use NLOPT to find the next subgoal for the trajectory generator
void find_intermediate_goal_params(EgoGoal *next_step)
{
  trajectory_count = 0;

  int max_iter = 250;  // 30
  nlopt::opt opt = nlopt::opt(nlopt::GN_DIRECT_NOSCAL, 4);
  opt.set_min_objective(score_trajectory, NULL);
  opt.set_xtol_rel(0.0001);
  std::vector<double> lb;
  std::vector<double> rb;
  lb.push_back(0);
  lb.push_back(-1.8);
  lb.push_back(-1.8);
  lb.push_back(V_MIN);
  rb.push_back(5.0);
  rb.push_back(1.8);
  rb.push_back(1.8);
  rb.push_back(V_MAX);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(rb);
  opt.set_maxeval(max_iter);

  std::vector<double> k(4);
  k[0] = 0.0;
  k[1] = 0.0;
  k[2] = 0.0;
  k[3] = 0.0;
  double minf;

  opt.optimize(k, minf);

  ROS_DEBUG("Global Optimization - Trajectories evaluated: %d", trajectory_count);
  trajectory_count = 0;

  max_iter = 75;  // 200
  nlopt::opt opt2 = nlopt::opt(nlopt::LN_BOBYQA, 4);
  opt2.set_min_objective(score_trajectory, NULL);
  opt2.set_xtol_rel(0.0001);
  std::vector<double> lb2;
  std::vector<double> rb2;
  lb2.push_back(0);
  lb2.push_back(-1.8);
  lb2.push_back(-3.1);
  lb2.push_back(V_MIN);
  rb2.push_back(5.0);
  rb2.push_back(1.8);
  rb2.push_back(3.1);
  rb2.push_back(V_MAX);
  opt2.set_lower_bounds(lb2);
  opt2.set_upper_bounds(rb2);
  opt2.set_maxeval(max_iter);

  opt2.optimize(k, minf);

  ROS_DEBUG("Local Optimization - Trajectories evaluated: %d", trajectory_count);
  trajectory_count = 0;

  next_step->r = k[0];
  next_step->delta = k[1];
  next_step->theta = k[2];
  next_step->vMax = k[3];
  next_step->k1 = K_1;
  next_step->k2 = K_2;

  return;
}

void nav_status_cb(const std_msgs::Int32::ConstPtr& status)
{
  ROS_DEBUG("Recieved Status: %d", status->data);
  current_status = *status;
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& nav_goal)
{
  ROS_DEBUG("Recieved Goal Request");
  global_goal_pose = *nav_goal;

  bHasGoal = true;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
  boost::mutex::scoped_lock lock(pose_mutex_);
  current_pose = *pose;
  current_pose_stamp.header = current_pose.header;
  current_pose_stamp.pose = current_pose.pose.pose;
  bHasOdom = true;
}

// Place new list of occupied cells in to a tree structure that supports fast
// nearest neighbor searching
void nav_cb(const nav_msgs::GridCells::ConstPtr& grid_cells)
{
  boost::mutex::scoped_lock lock(cost_map_mutex_);
  cost_map = *grid_cells;

  if(cost_map.cells.size() > 0)
  {
    delete obs_tree;
    delete data;
    data = new flann::Matrix<float>(new float[grid_cells->cells.size()*2], grid_cells->cells.size(), 2);

    for (size_t i = 0; i < data->rows; ++i)
    {
      for (size_t j = 0; j < data->cols; ++j)
      {
        if (j == 0)
          (*data)[i][j] = cost_map.cells[i].x;
        else
          (*data)[i][j] = cost_map.cells[i].y;
      }
    }
    // Obstacle index for fast nearest neighbor search
    obs_tree = new flann::Index<flann::L2<float> >(*data, flann::KDTreeIndexParams(4));
    obs_tree->buildIndex();
    bHasCostMap = true;
  }
}

geometry_msgs::PoseArray get_trajectory_viz(EgoGoal new_coords)
{
  geometry_msgs::PoseArray viz_plan;
  viz_plan.header.stamp = ros::Time::now();
  viz_plan.header.frame_id = "/odom";
  viz_plan.poses.resize(1);

  nav_msgs::Odometry sim_pose;

  {
    boost::mutex::scoped_lock lock(pose_mutex_);
    sim_pose = current_pose;
  }

  EgoPolar sim_goal;
  sim_goal.r = new_coords.r;
  sim_goal.delta = new_coords.delta;
  sim_goal.theta = new_coords.theta;

  geometry_msgs::Pose current_goal = cl->convert_from_egopolar(sim_pose, sim_goal);

  double sim_clock = 0.0;

  geometry_msgs::Twist sim_cmd_vel;
  double current_yaw = tf::getYaw(sim_pose.pose.pose.orientation);

  while (sim_clock < TIME_HORIZON)
  {
    sim_cmd_vel = cl->get_velocity_command(sim_goal, new_coords.k1, new_coords.k2, new_coords.vMax);

    // Update pose
    current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
    sim_pose.pose.pose.position.x = sim_pose.pose.pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
    sim_pose.pose.pose.position.y = sim_pose.pose.pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
    sim_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);
    viz_plan.poses.push_back(sim_pose.pose.pose);

    sim_goal = cl->convert_to_egopolar(sim_pose, current_goal);

    sim_clock = sim_clock + DELTA_SIM_TIME;
  }

  return viz_plan;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "mpepc_planner");
  ros::NodeHandle n;
  nh = &n;

  // Initialize Motion Model
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

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap_init("mpepc_costmap", tf);
  costmap = &costmap_init;
  nav_fn.initialize("mpepc_navfn", costmap);

  map_resolution = (costmap->getCostmap())->getResolution();
  interp_rotation_factor = (sqrt(2) * map_resolution - map_resolution)/2;

  ROS_DEBUG("map_resolution %f", map_resolution);

  double loop_rate = DEFAULT_LOOP_RATE;
  double global_loop_rate;
  if (ros::param::has("/global_loop_rate"))
  {
      nh->getParam("/global_loop_rate", global_loop_rate);
      loop_rate= RATE_FACTOR*global_loop_rate;
      ROS_DEBUG("mpepc_plan: Loop rate updated using global_loop_rate to : %f", loop_rate);
  }
  ros::Rate rate_obj(25.0);

  bHasGoal = false;
  bHasCostMap = false;
  bUseWaypointFollowing = false;
  current_status.data = RESULT_CANCEL;

  ego_goal_pub = nh->advertise<model_predictive_navigation::EgoGoal> ("int_goal_pose", 1);

  odom_sub = nh->subscribe<nav_msgs::Odometry>("/odom", 1, odom_cb);
  nav_result_sub = nh->subscribe<std_msgs::Int32> ("/nav_result", 1, nav_status_cb);
  goal_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/goal_request", 1, goal_cb);
  cost_map_sub = nh->subscribe<nav_msgs::GridCells>("/costmap_translator/obstacles", 10, nav_cb);  // cost

  plan_pub = nh->advertise<geometry_msgs::PoseArray>("/navfn_plan", 1);
  traj_pub = nh->advertise<geometry_msgs::PoseArray>("/traj_plan", 1);
  min_goal_obstacle_pub = nh->advertise<std_msgs::Float32MultiArray>("min_goal_obstacle_dist", 1);

  vector<geometry_msgs::PoseStamped> plan;
  std_msgs::Float32MultiArray minDist_goal_obstacle_f;
  minDist_goal_obstacle_f.data.resize(2);

  float minObstacleDist = 0.0;

  ros::AsyncSpinner aspin(4);
  aspin.start();
  // Control Loop and code
  while (ros::ok())
  {
    if (current_status.data == RESULT_BEGIN && bHasGoal && bHasCostMap && bHasOdom)
    {
      plan.clear();
      ROS_DEBUG("Begin Planning");
      EgoGoal new_coords;

      ROS_DEBUG("Compute Nav Function");
      nav_fn.computePotential(global_goal_pose.pose.position);
      geometry_msgs::PoseStamped temp_pose;

      {
        boost::mutex::scoped_lock lock(pose_mutex_);
        temp_pose.header = current_pose.header;
        temp_pose.pose = current_pose.pose.pose;
      }

      nav_fn.getPlanFromPotential(temp_pose, plan);

      if (!plan.empty())
      {
        ROS_DEBUG("Published Plan");
        geometry_msgs::PoseArray viz_plan;
        viz_plan.header = plan.front().header;
        for (int i = 0; i < plan.size(); i++)
        {
          viz_plan.poses.push_back(plan[i].pose);
        }
        plan_pub.publish(viz_plan);
      }
      else
      {
        ROS_DEBUG("Plan empty");
      }

      ROS_DEBUG("Optimize");
      double min_goal_dist = get_goal_distance();
      bUseWaypointFollowing = false;
      find_intermediate_goal_params(&new_coords);

      traj_pub.publish(get_trajectory_viz(new_coords));

      ROS_DEBUG("Publish");
      ego_goal_pub.publish(new_coords);
    }

    if (bHasGoal)
    {
      minDist_goal_obstacle_f.data[0] = get_goal_distance();
      minObstacleDist = get_obstacle_distance();
      minDist_goal_obstacle_f.data[1] = minObstacleDist;
      min_goal_obstacle_pub.publish(minDist_goal_obstacle_f);
    }

    rate_obj.sleep();
  }
}

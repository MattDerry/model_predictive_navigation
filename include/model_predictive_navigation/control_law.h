#ifndef CONTROLLAW_H
#define CONTROLLAW_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>

#define LINEAR_THRESHOLD 0.05
#define ANGULAR_THRESHOLD 0.05
#define R_SPEED_LIMIT 0.5

using namespace std;
namespace model_predictive_navigation
{
  struct EgoPolar
  {
      double r;
      double theta;
      double delta;
  };

  struct ControlLawSettings
  {
    ControlLawSettings() { m_K1 = 2.0; m_K2 = 3.0; m_BETA = 0.4; m_LAMBDA = 2.0; m_R_THRESH = 1.5; m_R_THRESH_TRANS = 1.0; m_TRANS_INTERVAL = 2.3; m_V_MAX = 0.3; m_V_TRANS = 0.2; m_V_MIN = 0.05;}
    public:
      double m_K1;
      double m_K2;              // K1 and K2 are parameters that shape the planned manifold
      double m_BETA;
      double m_LAMBDA;          // Beta and Lambda are parameters to shape the linear velocity rule
      double m_R_THRESH;
      double m_R_THRESH_TRANS;  // The thresholds of r to begin slowing down or switch to v_trans
      double m_TRANS_INTERVAL;  // Time spent to complete the transition
      double m_V_MAX;           // Max allowable linear velocity
      double m_V_TRANS;         // Constant velocity used for transition
      double m_V_MIN;           // Minimum velocity (not included in formulation, but used for practical reasons on real platform)
  };

  class ControlLaw
  {
    public:
      ControlLaw();
      explicit ControlLaw(ControlLawSettings c);
      geometry_msgs::Twist get_velocity_command(nav_msgs::Odometry current_position, geometry_msgs::Pose goal, double k1 = 2, double k2 = 3, double vMax = 0.3);
      geometry_msgs::Twist get_velocity_command(EgoPolar goal_coords, double k1, double k2, double vMax);
      geometry_msgs::Twist get_velocity_command(EgoPolar goal_coords, double vMax = 0.3);
      double get_ego_distance(nav_msgs::Odometry current_position, geometry_msgs::Pose goal);
      void update_k1_k2(double k1, double k2);
      EgoPolar convert_to_egopolar(nav_msgs::Odometry current_pose, geometry_msgs::Pose current_goal_pose);
      geometry_msgs::Pose convert_from_egopolar(nav_msgs::Odometry current_pose, EgoPolar current_goal_coords);
      double wrap_pos_neg_pi(double angle);

    protected:
      double get_kappa(EgoPolar current_ego_goal, double k1, double k2);
      double get_linear_vel(double kappa, EgoPolar current_ego_goal, double vMax);
      double get_angular_vel(double kappa, double linear_vel);
      double calc_sigmoid(double time_tau);

    private:
      static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
      static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

      double mod(double x, double y);


      ControlLawSettings settings_;
  };
}

#endif

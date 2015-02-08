#include <model_predictive_navigation/control_law.h>
#include <algorithm>

namespace model_predictive_navigation
{
  ControlLaw::ControlLaw()
  {
  }

  ControlLaw::ControlLaw(ControlLawSettings c)
  {
    settings_ = c;
  }

  // Floating-point modulo
  // The result (the remainder) has same sign as the divisor.
  // Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
  double ControlLaw::mod(double x, double y)
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
          return 0;    // just in case...
        else
          return y+m;  // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
      }
    }
    else                    // modulo range: (y..0]
    {
      if (m <= y)           // Mod(1e-16              , -360.   ): m= -360.
        return 0;

      if (m > 0)
      {
        if (y+m == y)
          return 0;    // just in case...
        else
          return y+m;  // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
      }
    }
    return m;
  }

  void ControlLaw::update_k1_k2(double k1, double k2)
  {
    settings_.m_K1 = k1;
    settings_.m_K2 = k2;
  }

  // wrap [rad] angle to [-PI..PI)
  double ControlLaw::wrap_pos_neg_pi(double angle)
  {
    return mod(angle + _PI, _TWO_PI) - _PI;
  }

  double ControlLaw::get_ego_distance(nav_msgs::Odometry current_pose, geometry_msgs::Pose current_goal_pose)
  {
    double distance;

    double dx = current_goal_pose.position.x - current_pose.pose.pose.position.x;
    double dy = current_goal_pose.position.y - current_pose.pose.pose.position.y;

    // calculate distance compenent of egocentric coordinates
    distance = sqrt(pow(dx, 2) + pow(dy, 2));

    return distance;
  }


  EgoPolar ControlLaw::convert_to_egopolar(nav_msgs::Odometry current_pose, geometry_msgs::Pose current_goal_pose)
  {
    EgoPolar coords;

    double dx = current_goal_pose.position.x - current_pose.pose.pose.position.x;
    double dy = current_goal_pose.position.y - current_pose.pose.pose.position.y;
    double obs_heading = atan2(dy, dx);
    double current_yaw = tf::getYaw(current_pose.pose.pose.orientation);
    double goal_yaw = tf::getYaw(current_goal_pose.orientation);

    // calculate r
    coords.r = sqrt(pow(dx, 2) + pow(dy, 2));
    // calculate delta
    coords.delta = wrap_pos_neg_pi(current_yaw - obs_heading);
    // calculate theta
    coords.theta = wrap_pos_neg_pi(goal_yaw - obs_heading);

    return coords;
  }

  geometry_msgs::Pose ControlLaw::convert_from_egopolar(nav_msgs::Odometry current_pose, EgoPolar current_goal_coords)
  {
    geometry_msgs::Pose current_goal_pose;

    double current_yaw = tf::getYaw(current_pose.pose.pose.orientation);

    current_goal_pose.position.x = current_pose.pose.pose.position.x + current_goal_coords.r * cos(current_yaw - current_goal_coords.delta);
    current_goal_pose.position.y = current_pose.pose.pose.position.y + current_goal_coords.r * sin(current_yaw - current_goal_coords.delta);
    current_goal_pose.position.z = 0;

    current_goal_pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw - current_goal_coords.delta + current_goal_coords.theta);

    return current_goal_pose;
  }

  double ControlLaw::get_kappa(EgoPolar current_ego_goal, double k1, double k2)
  {
    double kappa = 0;
    kappa = (-1/current_ego_goal.r)*(k2*(current_ego_goal.delta-atan(-1*k1*current_ego_goal.theta)) + (1 + k1/(1+pow(k1*current_ego_goal.theta, 2)))*sin(current_ego_goal.delta));
    return kappa;
  }

  double ControlLaw::get_linear_vel(double kappa, EgoPolar current_ego_goal, double vMax)
  {
    double lin_vel = 0;
    lin_vel = min(settings_.m_V_MAX/settings_.m_R_THRESH*current_ego_goal.r, settings_.m_V_MAX/(1 + settings_.m_BETA * pow(abs(kappa), settings_.m_LAMBDA)));

    if (lin_vel < settings_.m_V_MIN && lin_vel > 0.00)
    {
      lin_vel = settings_.m_V_MIN;
    }
    return lin_vel;
  }

  double ControlLaw::calc_sigmoid(double time_tau)
  {
    double sigma = 0;
    sigma = 1.02040816*(1/(1+exp(-9.2*(time_tau-0.5))) - 0.01);
    if (sigma > 1)
      sigma = 1;
    else if (sigma < 0)
      sigma = 0;
    return sigma;
  }

  geometry_msgs::Twist ControlLaw::get_velocity_command(nav_msgs::Odometry current_position, geometry_msgs::Pose goal, double k1, double k2, double vMax)
  {
    EgoPolar goal_coords = convert_to_egopolar(current_position, goal);

    return get_velocity_command(goal_coords, k1, k2, vMax);
  }

  geometry_msgs::Twist ControlLaw::get_velocity_command(EgoPolar goal_coords, double vMax)
  {
    return get_velocity_command(goal_coords, settings_.m_K1, settings_.m_K2, vMax);
  }

  geometry_msgs::Twist ControlLaw::get_velocity_command(EgoPolar goal_coords, double k1, double k2, double vMax)
  {
    geometry_msgs::Twist cmd_vel;
    double kappa = 0;

    kappa = get_kappa(goal_coords, k1, k2);

    cmd_vel.linear.x = get_linear_vel(kappa, goal_coords, vMax);
    cmd_vel.angular.z = kappa*cmd_vel.linear.x;
    if (fabs(cmd_vel.angular.z) > R_SPEED_LIMIT)
    {
      if (cmd_vel.angular.z > 0)
      {
        cmd_vel.angular.z = R_SPEED_LIMIT;
      }
      else
      {
        cmd_vel.angular.z = -1*R_SPEED_LIMIT;
      }
      cmd_vel.linear.x = cmd_vel.angular.z/kappa;
    }

    return cmd_vel;
  }
}  // end namespace

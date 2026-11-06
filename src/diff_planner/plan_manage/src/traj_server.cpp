#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double yaw_custom_;
double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;

bool receive_yaw_ = false;
ros::Time receive_yaw_time_(0);

// FOV visualization: follows the odometry pose so it matches the gsplat front camera.
ros::Publisher fov_vis_pub;
Eigen::Vector3d odom_pos_(Eigen::Vector3d::Zero());
double odom_yaw_ = 0.0;
bool has_odom_ = false;
double fov_top_angle_, fov_left_angle_, fov_right_angle_, fov_vis_dist_;
std::vector<Eigen::Vector3d> fov_vertices1_, fov_vertices2_;

// Velocity-colored flight history, same style as uss-nav's TrajectoryVisualizer.
ros::Publisher traj_vis_pub;
visualization_msgs::Marker traj_marker_;
Eigen::Vector3d last_traj_pos_(Eigen::Vector3d::Zero());
double traj_min_speed_, traj_max_speed_, traj_distance_threshold_;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void yawCallback(const quadrotor_msgs::PositionCommandPtr msg)
{
  receive_yaw_ = true;
  receive_yaw_time_ = ros::Time::now();
  yaw_custom_ = msg->yaw;
  // std::cout << "Received yaw:  " << yaw_custom_ << std::endl;
}

void initTrajMarker()
{
  traj_marker_.header.frame_id = "world";
  traj_marker_.header.stamp = ros::Time::now();
  traj_marker_.ns = "trajectory";
  traj_marker_.id = 0;
  traj_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  traj_marker_.action = visualization_msgs::Marker::ADD;
  traj_marker_.pose.orientation.w = 1.0;
  traj_marker_.scale.x = 0.05;
}

std_msgs::ColorRGBA hsvToRgb(double h, double s, double v)
{
  std_msgs::ColorRGBA rgb_color;
  rgb_color.a = 0.8;
  int i = floor(h * 6);
  double f = h * 6 - i;
  double p = v * (1 - s);
  double q = v * (1 - f * s);
  double t = v * (1 - (1 - f) * s);

  switch (i % 6)
  {
    case 0: rgb_color.r = v, rgb_color.g = t, rgb_color.b = p; break;
    case 1: rgb_color.r = q, rgb_color.g = v, rgb_color.b = p; break;
    case 2: rgb_color.r = p, rgb_color.g = v, rgb_color.b = t; break;
    case 3: rgb_color.r = p, rgb_color.g = q, rgb_color.b = v; break;
    case 4: rgb_color.r = t, rgb_color.g = p, rgb_color.b = v; break;
    case 5: rgb_color.r = v, rgb_color.g = p, rgb_color.b = q; break;
  }
  return rgb_color;
}

std_msgs::ColorRGBA speedToColor(double speed)
{
  double normalized_speed = (speed - traj_min_speed_) / (traj_max_speed_ - traj_min_speed_);
  normalized_speed = std::max(0.0, std::min(1.0, normalized_speed));
  double hue = (1.0 - normalized_speed) * 240.0 / 360.0;
  return hsvToRgb(hue, 1.0, 1.0);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  const auto &q = msg->pose.pose.orientation;
  odom_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  has_odom_ = true;

  // 轨迹点按最小间距采样，颜色按速度从蓝（慢）到红（快）。
  if ((odom_pos_ - last_traj_pos_).norm() > traj_distance_threshold_)
  {
    const geometry_msgs::Twist &tw = msg->twist.twist;
    const double speed = sqrt(tw.linear.x * tw.linear.x + tw.linear.y * tw.linear.y + tw.linear.z * tw.linear.z);

    geometry_msgs::Point p;
    p.x = odom_pos_(0);
    p.y = odom_pos_(1);
    p.z = odom_pos_(2);
    traj_marker_.points.push_back(p);
    traj_marker_.colors.push_back(speedToColor(speed));
    traj_marker_.header.stamp = ros::Time::now();
    traj_vis_pub.publish(traj_marker_);
    last_traj_pos_ = odom_pos_;
  }
}

void initFOVVertices()
{
  const double hor_left = fov_vis_dist_ * tan(fov_left_angle_);
  const double hor_right = fov_vis_dist_ * tan(fov_right_angle_);
  const double vert = fov_vis_dist_ * tan(fov_top_angle_);

  const Eigen::Vector3d origin(0.0, 0.0, 0.0);
  const Eigen::Vector3d left_up(fov_vis_dist_, hor_left, vert);
  const Eigen::Vector3d left_down(fov_vis_dist_, hor_left, -vert);
  const Eigen::Vector3d right_up(fov_vis_dist_, -hor_right, vert);
  const Eigen::Vector3d right_down(fov_vis_dist_, -hor_right, -vert);

  // 4 rays from the camera origin plus the 4 far-plane edges.
  fov_vertices1_ = {origin, origin, origin, origin, left_up, right_up, right_down, left_down};
  fov_vertices2_ = {left_up, left_down, right_up, right_down, right_up, right_down, left_down, left_up};
}

void drawFOV()
{
  if (!has_odom_ || fov_vertices1_.empty())
    return;

  const double cy = cos(odom_yaw_), sy = sin(odom_yaw_);
  Eigen::Matrix3d R;
  R << cy, -sy, 0.0,
       sy, cy, 0.0,
       0.0, 0.0, 1.0;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = "current_pose";
  mk.id = 0;
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;

  mk.points.resize(fov_vertices1_.size() * 2);
  for (size_t i = 0; i < fov_vertices1_.size(); ++i)
  {
    const Eigen::Vector3d p1 = R * fov_vertices1_[i] + odom_pos_;
    const Eigen::Vector3d p2 = R * fov_vertices2_[i] + odom_pos_;
    mk.points[2 * i + 0].x = p1(0);
    mk.points[2 * i + 0].y = p1(1);
    mk.points[2 * i + 0].z = p1(2);
    mk.points[2 * i + 1].x = p2(0);
    mk.points[2 * i + 1].y = p2(1);
    mk.points[2 * i + 1].z = p2(2);
  }
  fov_vis_pub.publish(mk);
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  if (receive_yaw_ && yaw_custom_ > -100.0)
  { 
    if ((ros::Time::now() - receive_yaw_time_).toSec() < 0.5)
    {
      yaw_temp = yaw_custom_;
    }
    else
    {
      receive_yaw_ = false;
    }
  }

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  // FOV 与轨迹状态无关：先按里程计位姿刷新（10 Hz），再处理指令。
  static ros::Time last_fov_pub(0);
  const ros::Time vis_now = ros::Time::now();
  if ((vis_now - last_fov_pub).toSec() > 0.1)
  {
    last_fov_pub = vis_now;
    drawFOV();
  }

  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, 0.01);
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
  else if (t_cur >= traj_duration_)
  {
    /* hold the final position and yaw after the trajectory ends */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0.0;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber yaw_sub = nh.subscribe("/planning/yaw", 10, yawCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  fov_vis_pub = nh.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  traj_vis_pub = nh.advertise<visualization_msgs::Marker>("/scene_graph/traj_with_vel", 10);
  ros::Subscriber odom_sub = nh.subscribe("odom", 50, odomCallback);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/yaw_dot_max", YAW_DOT_MAX_PER_SEC, YAW_DOT_MAX_PER_SEC);
  nh.param("traj_server/yaw_dot_dot_max", YAW_DOT_DOT_MAX_PER_SEC, YAW_DOT_DOT_MAX_PER_SEC);
  // Defaults follow uss-nav's perception_utils FOV parameters.
  nh.param("traj_server/fov_top_angle", fov_top_angle_, 0.6);
  nh.param("traj_server/fov_left_angle", fov_left_angle_, 0.76);
  nh.param("traj_server/fov_right_angle", fov_right_angle_, 0.76);
  nh.param("traj_server/fov_vis_dist", fov_vis_dist_, 1.0);
  nh.param("traj_server/traj_min_speed", traj_min_speed_, 0.0);
  nh.param("traj_server/traj_max_speed", traj_max_speed_, 1.5);
  nh.param("traj_server/traj_distance_threshold", traj_distance_threshold_, 0.1);
  initFOVVertices();
  initTrajMarker();
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
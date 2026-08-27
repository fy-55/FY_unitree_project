// Copyright (c) 2024 THEMIS
//
// SafeEscape: 综合多方向障碍物检测的安全逃逸/后退
//   - 扫描 360° 全方向，每个方向多深度采样
//   - 选择障碍物密度最低的方向
//   - 移动中实时检测，遇新障碍立即停止
//   - 支持后退偏好（用作安全 BackUp 替代）

#include "nav2_custom_plugins/escape_obstacle.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>

namespace nav2_custom_plugins
{

void EscapeObstacle::onConfigure()
{
  auto node = node_.lock();
  if (!node) return;

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node, behavior_name_ + ".escape_distance",     rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, behavior_name_ + ".scan_radius",         rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(node, behavior_name_ + ".scan_directions",     rclcpp::ParameterValue(24));
  declare_parameter_if_not_declared(node, behavior_name_ + ".scan_depth",          rclcpp::ParameterValue(4));
  declare_parameter_if_not_declared(node, behavior_name_ + ".scan_lateral_span",   rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(node, behavior_name_ + ".escape_speed",        rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, behavior_name_ + ".obstacle_threshold",  rclcpp::ParameterValue(128.0));
  declare_parameter_if_not_declared(node, behavior_name_ + ".prefer_back_ratio",   rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, behavior_name_ + ".check_ahead_dist",    rclcpp::ParameterValue(0.15));

  node->get_parameter(behavior_name_ + ".escape_distance",     escape_distance_);
  node->get_parameter(behavior_name_ + ".scan_radius",         scan_radius_);
  node->get_parameter(behavior_name_ + ".scan_directions",     scan_directions_);
  node->get_parameter(behavior_name_ + ".scan_depth",          scan_depth_);
  node->get_parameter(behavior_name_ + ".scan_lateral_span",   scan_lateral_span_);
  node->get_parameter(behavior_name_ + ".escape_speed",        escape_speed_);
  node->get_parameter(behavior_name_ + ".obstacle_threshold",  obstacle_threshold_);
  node->get_parameter(behavior_name_ + ".prefer_back_ratio",   prefer_back_ratio_);
  node->get_parameter(behavior_name_ + ".check_ahead_dist",    check_ahead_dist_);

  RCLCPP_INFO(logger_,
    "SafeEscape configured: dist=%.2f scan_r=%.2f dirs=%d depth=%d lat=%.2f "
    "speed=%.2f obs_th=%.0f back_bias=%.2f check_ahead=%.2f",
    escape_distance_, scan_radius_, scan_directions_, scan_depth_,
    scan_lateral_span_, escape_speed_, obstacle_threshold_,
    prefer_back_ratio_, check_ahead_dist_);
}

bool EscapeObstacle::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  auto node = node_.lock();
  if (!node) return false;

  try {
    auto tf_future = tf_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node->now();
    pose.pose.position.x = tf_future.transform.translation.x;
    pose.pose.position.y = tf_future.transform.translation.y;
    pose.pose.orientation = tf_future.transform.rotation;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(logger_, "TF lookup failed: %s", e.what());
    return false;
  }
}

bool EscapeObstacle::isDirectionSafe(
  double cx, double cy, double dir_global, double check_dist)
{
  if (!collision_checker_) return true;  // shutdown 保护
  double cos_d = std::cos(dir_global), sin_d = std::sin(dir_global);
  geometry_msgs::msg::Pose2D test_pose;
  test_pose.x = cx + cos_d * check_dist;
  test_pose.y = cy + sin_d * check_dist;
  test_pose.theta = dir_global;
  double score = collision_checker_->scorePose(test_pose, true);
  return score < obstacle_threshold_;
}

double EscapeObstacle::findSafestDirection(double cx, double cy, double yaw)
{
  if (!collision_checker_) return 0.0;  // shutdown 保护
  const int n = std::max(8, scan_directions_);
  const int n_depth = std::max(2, scan_depth_);
  const double lat_span = std::max(0.02, scan_lateral_span_);

  // robot_yaw 用于后退偏好: dir 偏离 robot_yaw 越远，偏置权重越大
  double back_dir = yaw + M_PI;
  while (back_dir > M_PI)  back_dir -= 2.0 * M_PI;
  while (back_dir < -M_PI) back_dir += 2.0 * M_PI;

  double best_dir = 0.0;
  double best_cost = 1e9;

  // ── 诊断: 记录每个方向的代价 ──
  struct DirCost { double dir; double cost; int hits; };
  std::vector<DirCost> all_costs;
  all_costs.reserve(n);

  bool first_costmap_refresh = true;  // 只刷新一次 costmap, 避免竞态

  for (int i = 0; i < n; ++i) {
    double dir = i * 2.0 * M_PI / n;
    double cos_dir = std::cos(dir), sin_dir = std::sin(dir);
    double perp_x = -sin_dir, perp_y = cos_dir;

    double cost = 0.0;
    int obstacle_hits = 0;

    // 多深度采样: 沿 dir 方向从近到远逐层检测
    for (int d = 0; d < n_depth; ++d) {
      double r = scan_radius_ * (0.25 + 0.75 * d / (n_depth - 1));

      for (int lat = 0; lat < 3; ++lat) {
        double offset = lat_span * (-1.0 + 1.0 * lat);  // -lat_span, 0, +lat_span
        geometry_msgs::msg::Pose2D test_pose;
        test_pose.x = cx + cos_dir * r + perp_x * offset;
        test_pose.y = cy + sin_dir * r + perp_y * offset;
        test_pose.theta = yaw;
        double sc = collision_checker_->scorePose(test_pose, first_costmap_refresh);
        first_costmap_refresh = false;

        // 命中障碍物: 计入惩罚
        if (sc >= obstacle_threshold_) {
          obstacle_hits++;
          cost += sc * (1.5 - 0.3 * d / (n_depth - 1));  // 近处障碍惩罚更重
        } else {
          cost += sc;
        }
      }
    }

    // 后退偏好: 如果该方向接近 robot_back_dir，略微降低代价
    if (prefer_back_ratio_ > 0.0) {
      double angle_to_back = std::abs(dir - back_dir);
      if (angle_to_back > M_PI) angle_to_back = 2.0 * M_PI - angle_to_back;
      // 越接近后退方向，减分越多 (prefer_back_ratio=1 时最多减 30%)
      double bias = prefer_back_ratio_ * 0.3 * (1.0 - angle_to_back / M_PI);
      cost *= (1.0 - bias);
    }

    // 所有采样点都被障碍物阻挡 → 大幅惩罚
    if (obstacle_hits >= n_depth * 2) {
      cost *= 3.0;
    }

    all_costs.push_back({dir, cost, obstacle_hits});

    if (cost < best_cost) {
      best_cost = cost;
      best_dir = dir;
    }
  }

  // ── 诊断日志: 列出 top-5 最安全方向及其障碍物命中数 ──
  std::sort(all_costs.begin(), all_costs.end(),
    [](const DirCost &a, const DirCost &b) { return a.cost < b.cost; });

  std::ostringstream diag;
  diag << std::fixed << std::setprecision(1);
  diag << "SafeEscape scan: pos=(" << cx << "," << cy << ") yaw=" << yaw * 180.0 / M_PI
       << "deg back_dir=" << back_dir * 180.0 / M_PI << "deg"
       << " r=" << scan_radius_ << " depth=" << n_depth
       << "\n  top5: ";
  for (int k = 0; k < std::min(5, n); ++k) {
    const auto &dc = all_costs[k];
    diag << "[" << dc.dir * 180.0 / M_PI << "deg c=" << dc.cost << " hits=" << dc.hits
         << (dc.dir == best_dir ? " ★" : "") << "] ";
  }

  int fully_blocked = 0;
  for (const auto &dc : all_costs) {
    if (dc.hits >= n_depth * 2) fully_blocked++;
  }
  diag << "\n  blocked_dirs=" << fully_blocked << "/" << n
       << " prefer_back=" << prefer_back_ratio_
       << " best=(" << best_dir * 180.0 / M_PI << "deg cost=" << best_cost << ")";

  RCLCPP_INFO(logger_, "%s", diag.str().c_str());

  return best_dir;
}

nav2_behaviors::Status EscapeObstacle::onRun(
  const std::shared_ptr<const DriveOnHeadingAction::Goal> command)
{
  (void)command;

  // ── 创建 vel_pub_ (父类 TimedBehavior 在 onRun 中创建, 需手动补上) ──
  {
    auto node = node_.lock();
    if (!node) return nav2_behaviors::Status::FAILED;
    if (!vel_pub_) {
      vel_pub_ = node->template create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS());
    }
  }

  if (node_.expired() || !collision_checker_) return nav2_behaviors::Status::FAILED;

  geometry_msgs::msg::PoseStamped pose;
  if (!getCurrentPose(pose)) {
    RCLCPP_WARN(logger_, "SafeEscape: cannot get current pose on start");
    return nav2_behaviors::Status::FAILED;
  }

  double cx = pose.pose.position.x;
  double cy = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);

  RCLCPP_INFO(logger_,
    "SafeEscape: ──────── ESCAPE START ────────");
  RCLCPP_INFO(logger_,
    "SafeEscape: pose=(%.3f,%.3f) yaw=%.1f deg",
    cx, cy, yaw * 180.0 / M_PI);

  // ── 检查当前位置的代价 ──
  {
    geometry_msgs::msg::Pose2D cur_pose;
    cur_pose.x = cx; cur_pose.y = cy; cur_pose.theta = yaw;
    double cur_score = collision_checker_->scorePose(cur_pose, true);
    RCLCPP_INFO(logger_,
      "SafeEscape: current pose cost=%.1f (threshold=%.0f) %s",
      cur_score, obstacle_threshold_,
      cur_score >= obstacle_threshold_ ? "⚠ BLOCKED" : "✓ CLEAR");
  }

  // 全方向扫描找最安全方向
  escape_yaw_ = findSafestDirection(cx, cy, yaw);
  start_x_ = cx;
  start_y_ = cy;
  escape_initialized_ = true;

  RCLCPP_INFO(logger_,
    "SafeEscape: dir=%.1f deg dist=%.2f speed=%.2f prefer_back=%.2f",
    escape_yaw_ * 180.0 / M_PI, escape_distance_, escape_speed_,
    prefer_back_ratio_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status EscapeObstacle::onCycleUpdate()
{
  // ── shutdown 保护: 节点已销毁时立即退出, 防止 SIGSEGV ──
  if (!escape_initialized_) return nav2_behaviors::Status::FAILED;
  if (node_.expired() || !collision_checker_) return nav2_behaviors::Status::FAILED;

  geometry_msgs::msg::PoseStamped pose;
  if (!getCurrentPose(pose)) {
    RCLCPP_WARN(logger_, "SafeEscape: TF lost during escape");
    return nav2_behaviors::Status::FAILED;
  }

  double cx = pose.pose.position.x;
  double cy = pose.pose.position.y;

  double dx = cx - start_x_;
  double dy = cy - start_y_;
  double moved = std::hypot(dx, dy);

  // ── 到达目标距离 → 成功 ──
  if (moved >= escape_distance_) {
    RCLCPP_INFO(logger_,
      "SafeEscape: ✓ DONE moved=%.3fm (target=%.3fm) "
      "from (%.3f,%.3f) to (%.3f,%.3f)",
      moved, escape_distance_, start_x_, start_y_, cx, cy);
    stopRobot();
    escape_initialized_ = false;
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // ── 实时安全检测: 前方有障碍物 → 提前终止 ──
  //   跳过前 2 帧 (0.2s), 给机器人时间脱离当前位置再检测
  //   否则 scan_radius < check_ahead_dist 会导致第一帧立刻判定堵住
  if (moved > 0.02 && !isDirectionSafe(cx, cy, escape_yaw_, check_ahead_dist_)) {
    geometry_msgs::msg::Pose2D cur_pose;
    cur_pose.x = cx; cur_pose.y = cy; cur_pose.theta = escape_yaw_;
    double cur_score = collision_checker_->scorePose(cur_pose, true);

    double ahead_x = cx + std::cos(escape_yaw_) * check_ahead_dist_;
    double ahead_y = cy + std::sin(escape_yaw_) * check_ahead_dist_;
    geometry_msgs::msg::Pose2D ahead_pose;
    ahead_pose.x = ahead_x; ahead_pose.y = ahead_y; ahead_pose.theta = escape_yaw_;
    double ahead_score = collision_checker_->scorePose(ahead_pose, true);

    RCLCPP_WARN(logger_,
      "SafeEscape: ✗ BLOCKED ahead @ %.2fm (dir=%.1f deg) "
      "cur_cost=%.1f ahead_cost=%.1f threshold=%.0f "
      "moved=%.3f/%.3f",
      check_ahead_dist_, escape_yaw_ * 180.0 / M_PI,
      cur_score, ahead_score, obstacle_threshold_,
      moved, escape_distance_);
    stopRobot();
    escape_initialized_ = false;
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // ── 侧向 + 后方障碍物检测: 如果其他方向突然出现障碍物，重新规划方向 ──
  {
    double yaw = tf2::getYaw(pose.pose.orientation);
    double current_dir = escape_yaw_;
    if (!isDirectionSafe(cx, cy, current_dir, scan_radius_ * 0.6)) {
      double new_dir = findSafestDirection(cx, cy, yaw);
      double angle_diff = std::abs(new_dir - current_dir);
      if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
      if (angle_diff > M_PI / 6.0) {  // >30°
        RCLCPP_INFO(logger_,
          "SafeEscape: ↻ REROUTE %.1f° → %.1f° (diff=%.1f°)",
          current_dir * 180.0 / M_PI, new_dir * 180.0 / M_PI,
          angle_diff * 180.0 / M_PI);
        escape_yaw_ = new_dir;
      }
    }
  }

  // ── 发布速度指令 ──
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  double c = std::cos(escape_yaw_), s = std::sin(escape_yaw_);
  double yaw_robot = tf2::getYaw(pose.pose.orientation);
  double cos_t = std::cos(-yaw_robot), sin_t = std::sin(-yaw_robot);
  cmd_vel->linear.x = (c * cos_t - s * sin_t) * escape_speed_;
  cmd_vel->linear.y = (c * sin_t + s * cos_t) * escape_speed_;
  cmd_vel->angular.z = 0.0;

  if (vel_pub_) {
    vel_pub_->publish(*cmd_vel);
  }

  RCLCPP_INFO(logger_,
    "SafeEscape: → vx=%.3f vy=%.3f dir=%.0f° moved=%.3f/%.3f (%.0f%%)",
    cmd_vel->linear.x, cmd_vel->linear.y,
    escape_yaw_ * 180.0 / M_PI,
    moved, escape_distance_, moved / escape_distance_ * 100.0);

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace nav2_custom_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_plugins::EscapeObstacle, nav2_core::Behavior)

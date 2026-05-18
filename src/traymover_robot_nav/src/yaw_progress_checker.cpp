#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace traymover_robot_nav
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class YawProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
  {
    plugin_name_ = plugin_name;
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("YawProgressChecker parent node is unavailable");
    }

    clock_ = node->get_clock();

    declare_if_missing(node, "required_movement_radius", 0.30);
    declare_if_missing(node, "required_movement_angle", 0.20);
    declare_if_missing(node, "movement_time_allowance", 8.0);

    node->get_parameter(plugin_name_ + ".required_movement_radius", radius_);
    node->get_parameter(plugin_name_ + ".required_movement_angle", angle_);

    double time_allowance = 8.0;
    node->get_parameter(plugin_name_ + ".movement_time_allowance", time_allowance);
    time_allowance_ = rclcpp::Duration::from_seconds(time_allowance);

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&YawProgressChecker::dynamic_parameters_callback, this, std::placeholders::_1));
  }

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override
  {
    const Pose2D pose = to_pose_2d(current_pose);
    if (!baseline_pose_set_ || moved_enough(pose) || rotated_enough(pose)) {
      reset_baseline_pose(pose);
      return true;
    }

    return !((clock_->now() - baseline_time_) > time_allowance_);
  }

  void reset() override
  {
    baseline_pose_set_ = false;
  }

private:
  void declare_if_missing(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    double value)
  {
    const std::string parameter_name = plugin_name_ + "." + name;
    if (!node->has_parameter(parameter_name)) {
      node->declare_parameter(parameter_name, rclcpp::ParameterValue(value));
    }
  }

  static Pose2D to_pose_2d(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;

    Pose2D pose_2d;
    pose_2d.x = pose.pose.position.x;
    pose_2d.y = pose.pose.position.y;
    pose_2d.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return pose_2d;
  }

  static double yaw_distance(double from, double to)
  {
    const double delta = to - from;
    return std::fabs(std::atan2(std::sin(delta), std::cos(delta)));
  }

  bool moved_enough(const Pose2D & pose) const
  {
    const double dx = pose.x - baseline_pose_.x;
    const double dy = pose.y - baseline_pose_.y;
    return std::hypot(dx, dy) > radius_;
  }

  bool rotated_enough(const Pose2D & pose) const
  {
    return yaw_distance(pose.yaw, baseline_pose_.yaw) > angle_;
  }

  void reset_baseline_pose(const Pose2D & pose)
  {
    baseline_pose_ = pose;
    baseline_time_ = clock_->now();
    baseline_pose_set_ = true;
  }

  static bool as_double(const rclcpp::Parameter & parameter, double & value)
  {
    switch (parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        value = parameter.as_double();
        return true;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        value = static_cast<double>(parameter.as_int());
        return true;
      default:
        return false;
    }
  }

  rcl_interfaces::msg::SetParametersResult dynamic_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      double value = 0.0;
      if (!as_double(parameter, value)) {
        continue;
      }

      if (name == plugin_name_ + ".required_movement_radius") {
        if (value < 0.0) {
          result.successful = false;
          result.reason = "required_movement_radius must be non-negative";
          return result;
        }
        radius_ = value;
      } else if (name == plugin_name_ + ".required_movement_angle") {
        if (value < 0.0) {
          result.successful = false;
          result.reason = "required_movement_angle must be non-negative";
          return result;
        }
        angle_ = value;
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        if (value <= 0.0) {
          result.successful = false;
          result.reason = "movement_time_allowance must be positive";
          return result;
        }
        time_allowance_ = rclcpp::Duration::from_seconds(value);
      }
    }

    return result;
  }

  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  double radius_{0.30};
  double angle_{0.20};
  rclcpp::Duration time_allowance_{0, 0};
  Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace traymover_robot_nav

PLUGINLIB_EXPORT_CLASS(traymover_robot_nav::YawProgressChecker, nav2_core::ProgressChecker)

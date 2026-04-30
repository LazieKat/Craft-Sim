#pragma once

#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "craft_sim/msg/vehicle_params.hpp"

namespace craft_sim
{

// Wrap angle to [-π, π]
inline double wrap_angle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

class AttitudeControllerNode : public rclcpp::Node
{
public:
  explicit AttitudeControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Gains
  double kp_{2.0}, ki_{0.05}, kd_{0.5}, ff_{0.0};

  // Latest inputs
  std::array<double, 3> desired_att_{0.0, 0.0, 0.0};
  std::array<double, 3> current_att_{0.0, 0.0, 0.0};

  // Controller state
  std::array<double, 3> error_prev_{0.0, 0.0, 0.0};
  std::array<double, 3> integral_{0.0, 0.0, 0.0};

  static constexpr double kDt            = 0.01;
  static constexpr double kIntegralClamp = 5.0;  // rad — anti-windup

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<craft_sim::msg::VehicleParams>::SharedPtr sub_params_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_desired_att_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_current_att_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_desired_rates_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_params(const craft_sim::msg::VehicleParams::SharedPtr msg);
  void on_desired_att(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void on_current_att(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void tick();
};

}  // namespace craft_sim

#pragma once

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "craft_sim/msg/vehicle_params.hpp"

namespace craft_sim
{

class RateControllerNode : public rclcpp::Node
{
public:
  explicit RateControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Gains
  double kp_{10.0}, kd_{0.5}, ff_{0.0};

  // Latest inputs
  std::array<double, 3> desired_rates_{0.0, 0.0, 0.0};
  std::array<double, 3> current_rates_{0.0, 0.0, 0.0};

  // Derivative state
  std::array<double, 3> error_prev_{0.0, 0.0, 0.0};

  static constexpr double kDt = 0.01;  // must match simulator

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<craft_sim::msg::VehicleParams>::SharedPtr sub_params_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_desired_rates_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_current_rates_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_torque_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_params(const craft_sim::msg::VehicleParams::SharedPtr msg);
  void on_desired_rates(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void on_current_rates(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void tick();
};

}  // namespace craft_sim

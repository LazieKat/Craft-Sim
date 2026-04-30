#pragma once

#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "craft_sim/msg/vehicle_params.hpp"

namespace craft_sim
{

class SimulatorNode : public rclcpp::Node
{
public:
  explicit SimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── State ──────────────────────────────────────────────────────────────────
  // [0]=roll [1]=pitch [2]=yaw  (rad, ZYX aerospace convention)
  std::array<double, 3> attitude_{0.0, 0.0, 0.0};
  // [0]=wx [1]=wy [2]=wz  (rad/s, body frame)
  std::array<double, 3> rates_{0.0, 0.0, 0.0};

  // Latest commands — updated from subscriber callbacks
  std::array<double, 3> torque_{0.0, 0.0, 0.0};
  double ixx_{1.0}, iyy_{2.0}, izz_{3.0};

  static constexpr double kDt = 0.01;  // 100 Hz

  // ── Physics ────────────────────────────────────────────────────────────────
  // Euler's equations: returns angular accelerations given current rates/torques
  std::array<double, 3> angular_accel(const std::array<double, 3> & w,
                                      const std::array<double, 3> & tau) const;

  // Kinematic equations: body rates → Euler angle rates (ZYX)
  std::array<double, 3> euler_rates(const std::array<double, 3> & angles,
                                    const std::array<double, 3> & w) const;

  // Single RK4 step over the full 6-DOF state [attitude | rates]
  void rk4_step();

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<craft_sim::msg::VehicleParams>::SharedPtr sub_params_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_attitude_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_rates_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_params(const craft_sim::msg::VehicleParams::SharedPtr msg);
  void on_torque(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void tick();
};

}  // namespace craft_sim

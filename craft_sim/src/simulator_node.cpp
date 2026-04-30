#include "craft_sim/simulator_node.hpp"

#include <algorithm>

namespace craft_sim
{

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions & options)
: Node("simulator_node", options)
{
  using std::placeholders::_1;

  sub_params_ = create_subscription<craft_sim::msg::VehicleParams>(
    "/vehicle_params", 10, std::bind(&SimulatorNode::on_params, this, _1));

  sub_torque_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/torque_commands", 10, std::bind(&SimulatorNode::on_torque, this, _1));

  pub_attitude_ = create_publisher<geometry_msgs::msg::Vector3>("/current_attitude", 10);
  pub_rates_    = create_publisher<geometry_msgs::msg::Vector3>("/current_rates",    10);

  auto period = std::chrono::duration<double>(kDt);
  timer_ = create_wall_timer(period, std::bind(&SimulatorNode::tick, this));

  RCLCPP_INFO(get_logger(), "Simulator started at %.0f Hz", 1.0 / kDt);
}

// ── Physics helpers ──────────────────────────────────────────────────────────

std::array<double, 3> SimulatorNode::angular_accel(
  const std::array<double, 3> & w,
  const std::array<double, 3> & tau) const
{
  // Euler's equations for a rigid body with diagonal inertia tensor:
  //   I·ω̇ = τ − ω × (I·ω)
  return {
    (tau[0] - (izz_ - iyy_) * w[1] * w[2]) / ixx_,
    (tau[1] - (ixx_ - izz_) * w[0] * w[2]) / iyy_,
    (tau[2] - (iyy_ - ixx_) * w[0] * w[1]) / izz_
  };
}

std::array<double, 3> SimulatorNode::euler_rates(
  const std::array<double, 3> & angles,
  const std::array<double, 3> & w) const
{
  // ZYX aerospace convention kinematic equations
  const double phi   = angles[0];
  const double theta = angles[1];
  const double sp = std::sin(phi),   cp = std::cos(phi);
  const double ct = std::cos(theta), tt = std::tan(theta);

  // Guard against gimbal lock (pitch ≈ ±90°)
  const double sec_theta = (std::abs(ct) > 1e-6) ? (1.0 / ct) : std::copysign(1e6, ct);

  return {
    w[0] + (w[1] * sp + w[2] * cp) * tt,
    w[1] * cp - w[2] * sp,
    (w[1] * sp + w[2] * cp) * sec_theta
  };
}

void SimulatorNode::rk4_step()
{
  // Full state: x = [phi, theta, psi, wx, wy, wz]
  // f(x) = [euler_rates(angles, w) | angular_accel(w, tau)]
  const auto & tau = torque_;

  auto f = [&](const std::array<double, 3> & ang,
               const std::array<double, 3> & w) ->
    std::pair<std::array<double, 3>, std::array<double, 3>>
  {
    return {euler_rates(ang, w), angular_accel(w, tau)};
  };

  auto add = [](const std::array<double, 3> & a,
                const std::array<double, 3> & b, double s) -> std::array<double, 3>
  {
    return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
  };

  // k1
  auto [da1, dw1] = f(attitude_, rates_);

  // k2
  auto ang2 = add(attitude_, da1, kDt * 0.5);
  auto w2   = add(rates_,    dw1, kDt * 0.5);
  auto [da2, dw2] = f(ang2, w2);

  // k3
  auto ang3 = add(attitude_, da2, kDt * 0.5);
  auto w3   = add(rates_,    dw2, kDt * 0.5);
  auto [da3, dw3] = f(ang3, w3);

  // k4
  auto ang4 = add(attitude_, da3, kDt);
  auto w4   = add(rates_,    dw3, kDt);
  auto [da4, dw4] = f(ang4, w4);

  constexpr double sixth = 1.0 / 6.0;
  for (int i = 0; i < 3; ++i) {
    attitude_[i] += (kDt * sixth) * (da1[i] + 2*da2[i] + 2*da3[i] + da4[i]);
    rates_[i]    += (kDt * sixth) * (dw1[i] + 2*dw2[i] + 2*dw3[i] + dw4[i]);
  }
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void SimulatorNode::on_params(const craft_sim::msg::VehicleParams::SharedPtr msg)
{
  ixx_ = (msg->ixx > 0.0) ? msg->ixx : ixx_;
  iyy_ = (msg->iyy > 0.0) ? msg->iyy : iyy_;
  izz_ = (msg->izz > 0.0) ? msg->izz : izz_;
}

void SimulatorNode::on_torque(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  torque_ = {msg->x, msg->y, msg->z};
}

void SimulatorNode::tick()
{
  rk4_step();

  geometry_msgs::msg::Vector3 att_msg;
  att_msg.x = attitude_[0];
  att_msg.y = attitude_[1];
  att_msg.z = attitude_[2];
  pub_attitude_->publish(att_msg);

  geometry_msgs::msg::Vector3 rate_msg;
  rate_msg.x = rates_[0];
  rate_msg.y = rates_[1];
  rate_msg.z = rates_[2];
  pub_rates_->publish(rate_msg);
}

}  // namespace craft_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<craft_sim::SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}

#include "craft_sim/rate_controller_node.hpp"

namespace craft_sim
{

RateControllerNode::RateControllerNode(const rclcpp::NodeOptions & options)
: Node("rate_controller_node", options)
{
  using std::placeholders::_1;

  sub_params_ = create_subscription<craft_sim::msg::VehicleParams>(
    "/vehicle_params", 10, std::bind(&RateControllerNode::on_params, this, _1));

  sub_desired_rates_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/desired_rates", 10, std::bind(&RateControllerNode::on_desired_rates, this, _1));

  sub_current_rates_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/current_rates", 10, std::bind(&RateControllerNode::on_current_rates, this, _1));

  pub_torque_ = create_publisher<geometry_msgs::msg::Vector3>("/torque_commands", 10);

  auto period = std::chrono::duration<double>(kDt);
  timer_ = create_wall_timer(period, std::bind(&RateControllerNode::tick, this));

  RCLCPP_INFO(get_logger(), "Rate controller started");
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void RateControllerNode::on_params(const craft_sim::msg::VehicleParams::SharedPtr msg)
{
  kp_ = msg->rate_kp;
  kd_ = msg->rate_kd;
  ff_ = msg->rate_ff;
}

void RateControllerNode::on_desired_rates(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  desired_rates_ = {msg->x, msg->y, msg->z};
}

void RateControllerNode::on_current_rates(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  current_rates_ = {msg->x, msg->y, msg->z};
}

void RateControllerNode::tick()
{
  geometry_msgs::msg::Vector3 out;

  for (int i = 0; i < 3; ++i) {
    const double e    = desired_rates_[i] - current_rates_[i];
    const double edot = (e - error_prev_[i]) / kDt;
    const double tau  = kp_ * e + kd_ * edot + ff_ * desired_rates_[i];
    error_prev_[i] = e;

    if (i == 0) out.x = tau;
    else if (i == 1) out.y = tau;
    else  out.z = tau;
  }

  pub_torque_->publish(out);
}

}  // namespace craft_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<craft_sim::RateControllerNode>());
  rclcpp::shutdown();
  return 0;
}

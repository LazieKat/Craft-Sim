#include "craft_sim/rate_controller.hpp"

namespace craft_sim
{

RateController::RateController(const rclcpp::NodeOptions & options)
: Node("rate_controller", options)
{
  using std::placeholders::_1;

  sub_params_ = create_subscription<craft_sim::msg::VehicleParams>(
    "/vehicle_params", 10, std::bind(&RateController::on_params, this, _1));

  sub_desired_rates_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/desired_rates", 10, std::bind(&RateController::on_desired_rates, this, _1));

  sub_current_rates_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/current_rates", 10, std::bind(&RateController::on_current_rates, this, _1));

  pub_torque_ = create_publisher<geometry_msgs::msg::Vector3>("/torque_commands", 10);

  auto period = std::chrono::duration<double>(kDt);
  timer_ = create_wall_timer(period, std::bind(&RateController::tick, this));

  RCLCPP_INFO(get_logger(), "Rate controller started");
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void RateController::on_params(const craft_sim::msg::VehicleParams::SharedPtr msg)
{
  kp_ = msg->rate_kp;
  kd_ = msg->rate_kd;
  ff_ = msg->rate_ff;
  rate_limits_[0] = msg->rate_limit_roll / 180 * M_PI;  // convert to rad/s
  rate_limits_[1] = msg->rate_limit_pitch / 180 * M_PI;
  rate_limits_[2] = msg->rate_limit_yaw / 180 * M_PI;
}

void RateController::on_desired_rates(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  desired_rates_ = {msg->x, msg->y, msg->z};

  for (int i = 0; i < 3; ++i) {
    if (desired_rates_[i] > rate_limits_[i])
    {
      desired_rates_[i] = rate_limits_[i];
    }
    else if (desired_rates_[i] < -rate_limits_[i])
    {
      desired_rates_[i] = -rate_limits_[i];
    } 
  }
}

void RateController::on_current_rates(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  current_rates_ = {msg->x, msg->y, msg->z};
}

void RateController::tick()
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
  rclcpp::spin(std::make_shared<craft_sim::RateController>());
  rclcpp::shutdown();
  return 0;
}

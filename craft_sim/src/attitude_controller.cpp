#include "craft_sim/attitude_controller.hpp"

#include <algorithm>

namespace craft_sim
{

AttitudeController::AttitudeController(const rclcpp::NodeOptions & options)
: Node("attitude_controller", options)
{
  using std::placeholders::_1;

  sub_params_ = create_subscription<craft_sim::msg::VehicleParams>(
    "/vehicle_params", 10, std::bind(&AttitudeController::on_params, this, _1));

  sub_desired_att_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/desired_attitude", 10, std::bind(&AttitudeController::on_desired_att, this, _1));

  sub_current_att_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/current_attitude", 10, std::bind(&AttitudeController::on_current_att, this, _1));

  pub_desired_rates_ = create_publisher<geometry_msgs::msg::Vector3>("/desired_rates", 10);

  auto period = std::chrono::duration<double>(kDt);
  timer_ = create_wall_timer(period, std::bind(&AttitudeController::tick, this));

  RCLCPP_INFO(get_logger(), "Attitude controller started");
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void AttitudeController::on_params(const craft_sim::msg::VehicleParams::SharedPtr msg)
{
  kp_ = msg->att_kp;
  ki_ = msg->att_ki;
  kd_ = msg->att_kd;
  ff_ = msg->att_ff;
}

void AttitudeController::on_desired_att(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  desired_att_ = {msg->x, msg->y, msg->z};
}

void AttitudeController::on_current_att(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  current_att_ = {msg->x, msg->y, msg->z};
}

void AttitudeController::tick()
{
  geometry_msgs::msg::Vector3 out;

  for (int i = 0; i < 3; ++i) {
    const double e = wrap_angle(desired_att_[i] - current_att_[i]);

    integral_[i] += e * kDt;
    integral_[i]  = std::clamp(integral_[i], -kIntegralClamp, kIntegralClamp);

    const double edot  = (e - error_prev_[i]) / kDt;
    const double w_des = kp_ * e + ki_ * integral_[i] + kd_ * edot + ff_ * desired_att_[i];
    error_prev_[i] = e;

    if (i == 0) out.x = w_des;
    else if (i == 1) out.y = w_des;
    else  out.z = w_des;
  }

  pub_desired_rates_->publish(out);
}

}  // namespace craft_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<craft_sim::AttitudeController>());
  rclcpp::shutdown();
  return 0;
}

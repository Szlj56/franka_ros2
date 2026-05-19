#include "franka_mpc_controllers/joint_impedance_sim_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>

namespace franka_mpc_controllers {

controller_interface::CallbackReturn
JointImpedanceSimController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("stiffness", {});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<std::vector<double>>("effort_limit", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceSimController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_   = get_node()->get_parameter("joints").as_string_array();
  K_             = get_node()->get_parameter("stiffness").as_double_array();
  D_             = get_node()->get_parameter("damping").as_double_array();
  effort_limit_  = get_node()->get_parameter("effort_limit").as_double_array();

  const size_t n = joint_names_.size();
  q_d_.assign(n, 0.0);

  // Subscribe to planner — depth 1 so we always act on the latest plan
  traj_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/franka_mpc/joint_trajectory", 1,
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      current_traj_  = msg;
      traj_index_    = 0;
      traj_start_    = get_node()->now();
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "JointImpedanceSimController configured — %zu joints", n);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointImpedanceSimController::on_activate(const rclcpp_lifecycle::State &)
{
  // Hold current position on activation — no jump
  for (size_t i = 0; i < joint_names_.size(); ++i)
    q_d_[i] = state_interfaces_[i * 2].get_value();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
JointImpedanceSimController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // Advance to the correct trajectory point based on elapsed time
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (current_traj_ && !current_traj_->points.empty()) {
      double elapsed = (time - traj_start_).seconds();
      for (size_t i = traj_index_; i < current_traj_->points.size(); ++i) {
        double t = rclcpp::Duration(current_traj_->points[i].time_from_start).seconds();
        if (t <= elapsed) traj_index_ = i;
        else break;
      }
      const auto & pt = current_traj_->points[traj_index_];
      for (size_t i = 0; i < joint_names_.size(); ++i)
        q_d_[i] = pt.positions[i];
    }
  }

  // Impedance law — mirrors your MuJoCo actuator exactly:
  //   tau = gainprm*(ctrl - q) + biasprm[2]*dq
  //       = K*(q_d - q)        - D*dq
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double q  = state_interfaces_[i * 2    ].get_value();
    const double dq = state_interfaces_[i * 2 + 1].get_value();
    double tau = K_[i] * (q_d_[i] - q) - D_[i] * dq;
    tau = std::clamp(tau, -effort_limit_[i], effort_limit_[i]);
    command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointImpedanceSimController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_)
    cfg.names.push_back(j + "/effort");
  return cfg;
}

controller_interface::InterfaceConfiguration
JointImpedanceSimController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

} // namespace franka_mpc_controllers

PLUGINLIB_EXPORT_CLASS(
  franka_mpc_controllers::JointImpedanceSimController,
  controller_interface::ControllerInterface)
#pragma once
#include "controller_interface/controller_interface.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <mutex>

namespace franka_mpc_controllers {

class JointImpedanceSimController
  : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()   const override;
  controller_interface::CallbackReturn on_init()                                       override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)  override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)   override;
  controller_interface::return_type   update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> K_, D_, effort_limit_, q_d_;

  trajectory_msgs::msg::JointTrajectory::SharedPtr current_traj_;
  size_t traj_index_{0};
  rclcpp::Time traj_start_;
  std::mutex traj_mutex_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
};

} // namespace franka_mpc_controllers
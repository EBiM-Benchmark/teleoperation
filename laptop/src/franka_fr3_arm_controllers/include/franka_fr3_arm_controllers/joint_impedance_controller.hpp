// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include "franka_fr3_arm_controllers/motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_fr3_arm_controllers {

/**
 * Controller to move the robot to a desired joint position.
 */
class JointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string namespace_prefix_;
  static constexpr int num_joints = 7;
  static constexpr double max_gello_state_age_seconds = 0.5;
  std::array<std::string, num_joints> expected_gello_joint_names_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double k_alpha_{0.99};
  double future_timestamp_tolerance_{0.05};
  Vector7d gello_joint_directions_{Vector7d::Ones()};
  Vector7d initial_robot_position_;
  Vector7d initial_gello_position_;
  bool mapping_references_valid_{false};
  // Speed of the one-off trajectory that brings the arm from wherever it is to the
  // GELLO's pose when teleoperation starts. Range (0, 1]; small is slow.
  double motion_generator_speed_factor_{0.05};
  // Cap on how fast the commanded goal may track the GELLO once following starts.
  // The GELLO can be moved far faster than the arm may safely follow; without a
  // limit a quick hand movement demands a large jump and trips a velocity reflex.
  // <= 0 disables the limit.
  double max_goal_velocity_{0.5};  // rad/s
  Vector7d q_goal_limited_;
  bool q_goal_limited_valid_{false};
  bool move_to_start_position_finished_{false};
  bool motion_generator_initialized_{false};
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_ = nullptr;
  bool gello_position_values_valid_ = false;
  std::array<double, 7> gello_position_values_{0, 0, 0, 0, 0, 0, 0};
  rclcpp::Time last_valid_gello_receive_time_;

  Vector7d calculateTauDGains_(const Vector7d& q_goal);
  Vector7d mapGelloToRobotGoal_();
  bool validateGains_(const std::vector<double>& gains, const std::string& gains_name);
  bool validateGelloJointState_(const sensor_msgs::msg::JointState& msg,
                                const rclcpp::Time& receive_time);
  bool hasFreshGelloState_() const;
  bool initializeMotionGenerator_();
  void updateJointStates_();
  void jointStateCallback_(const sensor_msgs::msg::JointState msg);
};

}  // namespace franka_fr3_arm_controllers

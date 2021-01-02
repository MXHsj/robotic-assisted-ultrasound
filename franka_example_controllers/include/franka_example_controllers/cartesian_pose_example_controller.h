// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

class CartesianPoseExampleController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  void target_pos_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void entry_pos_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  // robot states
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> current_pose_{};
  std::array<double, 12> target_pose_{};  // column major
  std::array<double, 12> entry_pose_{};   // column major
  // node handle & topics
  ros::NodeHandle nh_;
  ros::Subscriber target_msg;  // subscriber to eef target pose
  ros::Subscriber entry_msg;   // subscriber to eef entry pose
  // initial conditions
  double current_time = 0.0;
  double last_time = 0.0;
  bool isReachedWp = false;
  bool isReachedTar = false;
};

}  // namespace franka_example_controllers

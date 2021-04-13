// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
namespace franka_example_controllers {

class CartesianVelocityExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  void updateStatus();

  inline void isContact_callback(const std_msgs::Bool::ConstPtr& msg) { isContact = msg->data; }
  inline void cmd_pos_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (!isnan(msg->data[0])) {
      for (size_t i = 0; i < 12; i++) {
        target_pose_[i] = msg->data[i];
      }
    }
  }
  inline void cmd_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (!isnan(msg->data[0])) {
      for (size_t i = 0; i < 6; i++) {
        cmd_vel[i] = msg->data[i];
      }
    }
  }
  inline void cmd_js_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_js[0] = msg->linear.x;
    cmd_js[1] = msg->linear.y;
    cmd_js[2] = msg->linear.z;
    cmd_js[3] = msg->angular.x;
    cmd_js[4] = msg->angular.y;
    cmd_js[5] = msg->angular.z;
  }

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  // robot states
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> current_pose_{};
  std::array<double, 16> last_pose_{};
  std::array<double, 12> target_pose_{};   // column major
  std::array<double, 6> cmd_vel{};         // Vx Vy Vz Wx Wy Wz
  std::array<double, 6> cmd_js{};          // Vx Vy Vz Wx Wy Wz
  std::array<double, 6> last_command{};    // Vx Vy Vz Wx Wy Wz
  std::array<double, 6> current_wrench{};  // Fx Fy Fz Mx My Mz
  std::array<double, 6> last_wrench{};     // Fx Fy Fz Mx My Mz
  // node handle & topics
  ros::NodeHandle nh_;
  ros::Subscriber isContact_msg;  // subscribe to operating mode
  ros::Subscriber target_msg;     // subscribe to eef target pose
  ros::Subscriber cmd_vel_msg;    // subscribe to eef desired velocity
  ros::Subscriber cmd_js_msg;     // subscribe to joystick commands
  ros::Subscriber entry_msg;      // subscribe to eef entry pose
  // params
  double current_time;
  double last_time;
  bool isContact;
};

}  // namespace franka_example_controllers

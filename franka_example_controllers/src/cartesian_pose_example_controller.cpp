// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace franka_example_controllers {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // CHECK IF IN READY-POSITION - COMMENTED
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "CartesianPoseExampleController: Robot is not in the expected starting position for "
    //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch
    //         " "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::target_pos_callback(
    const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (!isnan(msg->data[0])) {
    for (size_t i = 0; i < 12; i++) {
      target_pose_[i] = msg->data[i];
    }
  }
  // std::cout << target_pose_[i] << " ";
  // std::cout << std::endl;
}

void CartesianPoseExampleController::entry_pos_callback(
    const std_msgs::Float64MultiArray::ConstPtr& msg) {
  for (size_t i = 0; i < 12; i++) {
    entry_pose_[i] = msg->data[i];
    // std::cout << entry_pose_[i] << " ";
  }
  // std::cout << std::endl;
}

void updatePose(ros::Duration& totalTimeElapse,
                std::array<double, 16>& curr_pose_,
                std::array<double, 12>& tar_pose_) {
  std::cout << "----------status update----------" << std::endl;
  std::cout << totalTimeElapse << "[s] current pose:" << std::endl;
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      std::cout << curr_pose_[col + row * 4] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "target pose:" << std::endl;
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 4; row++) {
      std::cout << tar_pose_[col + row * 3] << " ";
    }
    std::cout << std::endl;
  }
}

void calcNewPose(const std::array<double, 12>& curr_goal,
                 const std::array<double, 16>& curr_pose,
                 std::array<double, 16>& cmd_pose,
                 bool& reachedFlag) {
  // params
  double rot_increment = 1e-5;       // 8*1e-7
  double tran_increment = 1e-5;      // 8*1e-7
  double trans_goal_thresh = 0.001;  // higher accuracy in translation
  double rot_goal_thresh = 0.008;    // less accuracy in rotation
  // flags
  bool isReachedRotX = true;
  bool isReachedRotY = true;
  bool isReachedRotZ = true;
  bool isReachedTrans = true;

  if (curr_goal[0] != -1) {
    // rot x
    // for (int ix = 0; ix < 3; ix++) {
    //   if (std::abs(curr_goal[ix] - curr_pose[ix]) > rot_goal_thresh) {
    //     // cmd_pose[0] += copysign(increment, (target_pose_[0] - current_pose_[0]));
    //     // cmd_pose[1] += copysign(increment, (target_pose_[1] - current_pose_[1]));
    //     // cmd_pose[2] += copysign(increment, (target_pose_[2] - current_pose_[2]));
    //     cmd_pose[ix] += copysign(rot_increment, (curr_goal[ix] - curr_pose[ix]));
    //     isReachedRotX = false;
    //   }
    // }
    // rot y
    // for (int iy = 3; iy < 6; iy++) {
    //   if (std::abs(curr_goal[iy] - curr_pose[iy + 1]) > rot_goal_thresh) {
    //     // cmd_pose[4] += copysign(increment, (target_pose_[3] - current_pose_[4]));
    //     // cmd_pose[5] += copysign(increment, (target_pose_[4] - current_pose_[5]));
    //     // cmd_pose[6] += copysign(increment, (target_pose_[5] - current_pose_[6]));
    //     cmd_pose[iy + 1] += copysign(rot_increment, (curr_goal[iy] - curr_pose[iy + 1]));
    //     isReachedRotY = false;
    //   }
    // }
    // rot z
    // for (int iz = 6; iz < 9; iz++) {
    //   if (std::abs(curr_goal[iz] - curr_pose[iz + 2]) > rot_goal_thresh) {
    //     // cmd_pose[8] += copysign(increment, (target_pose_[6] - current_pose_[8]));
    //     // cmd_pose[9] += copysign(increment, (target_pose_[7] - current_pose_[9]));
    //     // cmd_pose[10] += copysign(increment, (target_pose_[8] - current_pose_[10]));
    //     cmd_pose[iz + 2] += copysign(rot_increment, (curr_goal[iz] - curr_pose[iz + 2]));
    //     isReachedRotZ = false;
    //   }
    // }
    // Px; Py; Pz
    for (int it = 9; it < 12; it++) {
      if (std::abs(curr_goal[it] - curr_pose[it + 3]) > trans_goal_thresh) {
        // cmd_pose[12] += copysign(increment, (target_pose_[9] - current_pose_[12]));
        // cmd_pose[13] += copysign(increment, (target_pose_[10] - current_pose_[13]));
        // cmd_pose[14] += copysign(increment, (target_pose_[11] - current_pose_[14]));
        cmd_pose[it + 3] += copysign(tran_increment, (curr_goal[it] - curr_pose[it + 3]));
        isReachedTrans = false;
      }
    }
  }

  if (isReachedRotX && isReachedRotY && isReachedRotZ && isReachedTrans) {
    reachedFlag = true;
  }
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  last_pose_ = initial_pose_;
  target_pose_ = {{initial_pose_[0], initial_pose_[1], initial_pose_[2], initial_pose_[4],
                   initial_pose_[5], initial_pose_[6], initial_pose_[8], initial_pose_[9],
                   initial_pose_[10], initial_pose_[12], initial_pose_[13], initial_pose_[14]}};
  elapsed_time_ = ros::Duration(0.0);
  target_msg =
      nh_.subscribe("target_pose", 1, &CartesianPoseExampleController::target_pos_callback, this);
  // entry_msg =
  //     nh_.subscribe("entry_pose", 1, &CartesianPoseExampleController::entry_pos_callback, this);
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  current_time = elapsed_time_.toSec();
  // std::cout << current_time << std::endl;

  // -------------------------------------------------------------------------
  // double radius = 0.3;
  // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);
  // std::array<double, 16> new_pose = initial_pose_;
  // new_pose[12] -= delta_x;
  // new_pose[14] -= delta_z;
  // cartesian_pose_handle_->setCommand(new_pose);
  // -------------------------------------------------------------------------

  if (current_time - last_time > 5.0) {  // report current pose
    updatePose(elapsed_time_, current_pose_, target_pose_);
    // std::cout << "Entry point: " << isReachedWp << " Target: " << isReachedTar << std::endl;
    last_time = current_time;
  }

  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  std::array<double, 16> new_pose = current_pose_;

  // if ((!isReachedWp) && (!isReachedTar)) {
  //   calcNewPose(entry_pose_, current_pose_, new_pose, isReachedWp);  // go to entry point
  // } else if ((isReachedWp) && (!isReachedTar)) {
  //   calcNewPose(target_pose_, current_pose_, new_pose, isReachedTar);  // go to target
  // } else if ((isReachedWp) && (isReachedTar)) {
  //   std::cout << "goal reached !!" << std::endl;
  //   isReachedWp = false;
  // }

  // double Px5 = 1.6889e-10;
  // double Px4 = -4.222e-8;
  // double Px3 = 2.8148e-6;
  // double Px2 = 9.7273e-18;
  // double Px1 = -1.7166e-16;
  // double Px0 = 0.2645;

  // double new_x = Px5 * current_time * current_time * current_time * current_time * current_time +
  //                Px4 * current_time * current_time * current_time * current_time +
  //                Px3 * current_time * current_time * current_time +
  //                Px2 * current_time * current_time + Px1 * current_time + Px0;

  // new_pose[12] = new_x;

  // new_pose[0] = target_pose_[0];
  // new_pose[1] = target_pose_[1];
  // new_pose[2] = target_pose_[2];

  // new_pose[4] = target_pose_[3];
  // new_pose[5] = target_pose_[4];
  // new_pose[6] = target_pose_[5];

  // new_pose[8] = target_pose_[6];
  // new_pose[9] = target_pose_[7];
  // new_pose[10] = target_pose_[8];

  new_pose[12] = target_pose_[9];
  new_pose[13] = target_pose_[10];
  new_pose[14] = target_pose_[11];

  new_pose[12] = (7 / 8) * target_pose_[9] + (1 - 7 / 8) * last_pose_[12];
  new_pose[13] = (7 / 8) * target_pose_[10] + (1 - 7 / 8) * last_pose_[13];
  new_pose[14] = (7 / 8) * target_pose_[11] + (1 - 7 / 8) * last_pose_[14];
  last_pose_ = current_pose_;

  // calcNewPose(target_pose_, current_pose_, new_pose, isReachedTar);  // go to target
  cartesian_pose_handle_->setCommand(new_pose);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
// #include <math/YawPitchRoll.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#define rad2deg 180 / 3.1415

namespace franka_example_controllers {

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // CHECK IF IN READY-POSITION - COMMENTED
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "CartesianVelocityExampleController: Robot is not in the expected starting position "
    //         "for running this example. Run `roslaunch franka_example_controllers "
    //         "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
    //         "first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityExampleController::updatePose(ros::Duration& totalTimeElapse,
                                                    std::array<double, 16>& curr_pose_) {
  std::cout << totalTimeElapse << "[s] current pose:" << std::endl;
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      std::cout << curr_pose_[col + row * 4] << " ";
    }
    std::cout << std::endl;
  }
}

std::array<double, 3> rot2rpy(const std::array<double, 9>& rot_mat) {
  // approach from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
  std::array<double, 3> rpy{{0.0, 0.0, 0.0}};  // roll, pitch, yaw
  double sy = std::sqrt(rot_mat[5] * rot_mat[5] + rot_mat[8] * rot_mat[8]);
  bool singular = sy < 1e-6;
  if (!singular) {
    rpy[0] = atan2(rot_mat[5], rot_mat[8]);
    rpy[1] = atan2(-rot_mat[2], sy);
    rpy[2] = atan2(rot_mat[1], rot_mat[0]);
  } else {
    rpy[0] = atan2(rot_mat[7], rot_mat[4]);
    rpy[1] = atan2(-rot_mat[2], sy);
    rpy[2] = 0;
  }
  // explicitly set sign for roll
  if (rot_mat[2] < 0) {
    rpy[0] = copysign(rpy[0], 1);
  } else if (rot_mat[2] > 0) {
    rpy[0] = copysign(rpy[0], -1);
  }
  return rpy;
}

std::array<double, 6> calcNewVel(const std::array<double, 12>& curr_goal,
                                 const std::array<double, 16>& curr_pose,
                                 const std::array<double, 16>& prev_pose,
                                 const std::array<double, 6>& prev_vel,
                                 const ros::Duration& dt,
                                 bool& reachedFlag) {
  // take 3x3 rotation from 4x4 transformation
  std::array<double, 9> goal_rot{{curr_goal[0], curr_goal[1], curr_goal[2], curr_goal[3],
                                  curr_goal[4], curr_goal[5], curr_goal[6], curr_goal[7],
                                  curr_goal[8]}};
  std::array<double, 9> curr_rot{{curr_pose[0], curr_pose[1], curr_pose[2], curr_pose[4],
                                  curr_pose[5], curr_pose[6], curr_pose[8], curr_pose[9],
                                  curr_pose[10]}};
  std::array<double, 9> prev_rot{{prev_pose[0], prev_pose[1], prev_pose[2], prev_pose[4],
                                  prev_pose[5], prev_pose[6], prev_pose[8], prev_pose[9],
                                  prev_pose[10]}};

  // convert 3x3 rotation to roll pitch yaw w.r.t base
  auto curr_rpy = rot2rpy(curr_rot);
  auto prev_rpy = rot2rpy(prev_rot);
  auto goal_rpy = rot2rpy(goal_rot);

  // errors
  double e_v_x = curr_goal[9] - curr_pose[12];
  double e_v_y = curr_goal[10] - curr_pose[13];
  double e_v_z = curr_goal[11] - curr_pose[14];
  double e_w_x = (goal_rpy[0] - curr_rpy[0]) * rad2deg;
  double e_w_y = (goal_rpy[1] - curr_rpy[1]) * rad2deg;
  double e_w_z = (goal_rpy[2] - curr_rpy[2]) * rad2deg;
  double de_w_x = prev_vel[3] - (curr_rpy[0] - prev_rpy[0]) / dt.toSec();
  double de_w_y = prev_vel[4] - (curr_rpy[1] - prev_rpy[1]) / dt.toSec();
  double de_w_z = prev_vel[5] - (curr_rpy[2] - prev_rpy[2]) / dt.toSec();

  // params
  double linear_ramp = 0.00001;   // linear velocity increment
  double angular_ramp = 0.00003;  // angular velocity increment
  double linear_max = 0.28;       // maximum linear velocity
  double angular_max = 0.05;      // maximum angular velocity
  double tol_v_x = 0.003;         // x tolerance
  double tol_v_y = 0.003;
  double tol_v_z = 0.003;
  double tol_w_x = 0.02 * rad2deg;  // roll tolerance 0.08
  double tol_w_y = 0.02 * rad2deg;  // pitch tolerance 0.08
  double tol_w_z = 0.02 * rad2deg;  // yaw tolerance 0.08

  // PD velocity command
  double desired_v_x = 0.1 * e_v_x;
  double desired_v_y = 0.1 * e_v_y;
  double desired_v_z = 0.1 * e_v_z;
  double desired_w_x = 0.00035 * e_w_x + 0.6 * de_w_x;  // kp = 0.015, kd = 0.6
  double desired_w_y = 0.0008 * e_w_y + 0.2 * de_w_y;   // kp = 0.014, kd = 0.3
  double desired_w_z = 0.0008 * e_w_z + 0.2 * de_w_z;   // kp = 0.014, kd = 0.3
  auto cmd_vel = prev_vel;

  if (std::abs(cmd_vel[0]) <= linear_max) {
    cmd_vel[0] += copysign(linear_ramp, desired_v_x - prev_vel[0]);
  }

  if (std::abs(cmd_vel[1]) <= linear_max) {
    cmd_vel[1] += copysign(linear_ramp, desired_v_y - prev_vel[1]);
  }

  if (std::abs(cmd_vel[2]) <= linear_max) {
    cmd_vel[2] += copysign(linear_ramp, desired_v_z - prev_vel[2]);
  }

  if (desired_w_x != prev_vel[3]) {
    cmd_vel[3] += copysign(angular_ramp, desired_w_x - prev_vel[3]);
  } else {
    cmd_vel[3] += 0.0;
  }

  if (desired_w_y != prev_vel[4]) {
    cmd_vel[4] += copysign(angular_ramp, desired_w_y - prev_vel[4]);
  } else {
    cmd_vel[4] += 0.0;
  }

  if (desired_w_z != prev_vel[5]) {
    cmd_vel[5] += copysign(angular_ramp, desired_w_z - prev_vel[5]);
  } else {
    cmd_vel[5] += 0.0;
  }

  bool isReachedVx = (std::abs(e_v_x) > tol_v_x) ? false : true;
  bool isReachedVy = (std::abs(e_v_y) > tol_v_y) ? false : true;
  bool isReachedVz = (std::abs(e_v_z) > tol_v_z) ? false : true;
  bool isReachedWx = (std::abs(e_w_x) > tol_w_x) ? false : true;
  bool isReachedWy = (std::abs(e_w_y) > tol_w_y) ? false : true;
  bool isReachedWz = (std::abs(e_w_z) > tol_w_z) ? false : true;
  if (isReachedVx && isReachedVy && isReachedVz && isReachedWx && isReachedWy && isReachedWz) {
    reachedFlag = true;
  }

  std::cout << "--------------------------INFO-----------------------------" << std::endl;
  std::cout << "e_v_x:" << e_v_x << " e_v_y:" << e_v_y << " e_v_z:" << e_v_z << std::endl;
  std::cout << "e_w_x:" << e_w_x << " e_w_y:" << e_w_y << " e_w_z:" << e_w_z << std::endl;

  std::cout << "roll:" << goal_rpy[0] * rad2deg << " pitch:" << goal_rpy[1] * rad2deg
            << " yaw:" << goal_rpy[2] * rad2deg << std::endl;
  std::cout << "roll:" << curr_rpy[0] * rad2deg << " pitch:" << curr_rpy[1] * rad2deg
            << " yaw:" << curr_rpy[2] * rad2deg << std::endl;

  std::cout << "w_x:" << cmd_vel[3] << " w_y:" << cmd_vel[4] << " w_z:" << cmd_vel[5] << std::endl;
  std::cout << "-----------------------------------------------------------" << std::endl;

  return cmd_vel;
}  // namespace franka_example_controllers

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;
  last_pose_ = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);
  double current_time = 0.0;
  double last_time = 0.0;
  bool isReachedWp = false;
  bool isReachedTar = false;
  last_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  target_msg = nh_.subscribe("target_pose", 1,
                             &CartesianVelocityExampleController::target_pos_callback, this);
  entry_msg =
      nh_.subscribe("entry_pose", 1, &CartesianVelocityExampleController::entry_pos_callback, this);
}

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  // -------------------------------------------------------------------------
  elapsed_time_ += period;
  current_time = elapsed_time_.toSec();

  // double time_max = 4.0;
  // double v_max = 0.05;
  // double angle = M_PI / 4.0;
  // double cycle = std::floor(
  //     pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) /
  //     time_max));
  // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max *
  // elapsed_time_.toSec())); double v_x = std::cos(angle) * v; double v_z = -std::sin(angle) * v;
  // std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  // velocity_cartesian_handle_->setCommand(command);
  // -------------------------------------------------------------------------

  current_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

  std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  if ((!isReachedWp) && (!isReachedTar)) {
    command = calcNewVel(entry_pose_, current_pose_, last_pose_, last_command, period, isReachedWp);
  } else if ((isReachedWp) && (!isReachedTar)) {
    command =
        calcNewVel(target_pose_, current_pose_, last_pose_, last_command, period, isReachedTar);
  } else if (isReachedWp && isReachedTar) {
    std::cout << "goal reached!";
    isReachedWp = false;
  }

  velocity_cartesian_handle_->setCommand(command);
  last_command = command;
  last_pose_ = current_pose_;

  // if (current_time - last_time > 10.0) {  // report current status
  //   updatePose(elapsed_time_, current_pose_);
  //   std::cout << "Entry point: " << isReachedWp << " Target: " << isReachedTar << std::endl;
  //   last_time = current_time;
  // }
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)

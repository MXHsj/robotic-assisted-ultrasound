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

void CartesianVelocityExampleController::updateStatus() {
  std::cout << "----------status update----------" << std::endl;
  std::cout << "total time elapsed:" << elapsed_time_ << "[s]" << std::endl;
  std::cout << "current pose:" << std::endl;
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      std::cout << current_pose_[col + row * 4] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "target pose:" << std::endl;
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 4; row++) {
      std::cout << target_pose_[col + row * 3] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
  std::cout << "current eef wrench:" << std::endl;
  std::cout << "Fx: " << current_wrench[0] << " Fy: " << current_wrench[1]
            << " Fz: " << current_wrench[2] << " Mx: " << current_wrench[3]
            << " My: " << current_wrench[4] << " Mz: " << current_wrench[5] << std::endl;
  std::cout << "last velocity command:" << std::endl;
  std::cout << "vx: " << last_command[0] << " vy: " << last_command[1] << " vz: " << last_command[2]
            << " wx: " << last_command[3] << " wy: " << last_command[4]
            << " wz: " << last_command[5] << std::endl;
  std::cout << "isContact: " << isContact << std::endl;
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
  // if (rot_mat[2] < 0) {
  //   rpy[0] = copysign(rpy[0], 1);
  // } else if (rot_mat[2] > 0) {
  //   rpy[0] = copysign(rpy[0], -1);
  // }
  // // // explicitly set sign for pitch
  // if (rot_mat[6] > 0) {
  //   rpy[1] = copysign(rpy[1], -1);
  // } else if (rot_mat[6] < 0) {
  //   rpy[1] = copysign(rpy[1], 1);
  // }
  rpy[0] = (rpy[0] < 0) ? rpy[0] + 6.28 : rpy[0];
  return rpy;
}

std::array<double, 3> forceControl(const std::array<double, 12>& curr_goal,
                                   const std::array<double, 6>& curr_wrench,
                                   float d_Fz) {
  // translational component
  std::array<double, 3> P0{{curr_goal[9], curr_goal[10], curr_goal[11]}};
  // approach vector
  std::array<double, 3> Vz{{curr_goal[6], curr_goal[7], curr_goal[8]}};
  double F_eef_desired = d_Fz;
  double F_eef = curr_wrench[2];
  // double F_eef = sqrt(curr_wrench[0] * curr_wrench[0] + curr_wrench[1] * curr_wrench[1] +
  //                     curr_wrench[2] * curr_wrench[2]);
  double d = 0.038 * (F_eef_desired - F_eef);  // force control factor 0.05
  // new translational component
  std::array<double, 3> Pz{{P0[0] + Vz[0] * d, P0[1] + Vz[1] * d, P0[2] + Vz[2] * d}};
  // std::cout << "F_eef:" << F_eef << " d: " << d << std::endl;
  return Pz;
}

std::array<double, 6> calcNewVel(const std::array<double, 12>& curr_goal,
                                 const std::array<double, 16>& curr_pose,
                                 const std::array<double, 16>& prev_pose,
                                 const std::array<double, 6>& prev_vel,
                                 const ros::Duration& dt) {
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
  double de_v_x = prev_vel[0] - (curr_pose[12] - prev_pose[12]) / dt.toSec();
  double de_v_y = prev_vel[1] - (curr_pose[13] - prev_pose[13]) / dt.toSec();
  double de_v_z = prev_vel[2] - (curr_pose[14] - prev_pose[14]) / dt.toSec();

  double e_w_x = (goal_rpy[0] - curr_rpy[0]) * rad2deg;
  double e_w_y = (goal_rpy[1] - curr_rpy[1]) * rad2deg;
  double e_w_z = (goal_rpy[2] - curr_rpy[2]) * rad2deg;
  double de_w_x = prev_vel[3] - (curr_rpy[0] - prev_rpy[0]) / dt.toSec();
  double de_w_y = prev_vel[4] - (curr_rpy[1] - prev_rpy[1]) / dt.toSec();
  double de_w_z = prev_vel[5] - (curr_rpy[2] - prev_rpy[2]) / dt.toSec();

  // params
  double linear_ramp = 0.00002;   // linear velocity increment
  double angular_ramp = 0.00002;  // angular velocity increment
  double linear_max = 0.2;        // maximum linear velocity
  double angular_max = 0.02;      // maximum angular velocity

  // PD velocity command
  double desired_v_x = 0.1 * e_v_x + 0.1 * de_v_x;      // kp = 0.1
  double desired_v_y = 0.1 * e_v_y + 0.1 * de_v_y;      // kp = 0.1
  double desired_v_z = 0.08 * e_v_z + 0.65 * de_v_z;    // kp = 0.35, kd = 0.65
  double desired_w_x = 0.0003 * e_w_x + 0.64 * de_w_x;  // kp = 0.0002, kd = 0.8
  double desired_w_y = 0.0015 * e_w_y + 0.2 * de_w_y;   // kp = 0.0008, kd = 0.2
  double desired_w_z = 0.0015 * e_w_z + 0.2 * de_w_z;   // kp = 0.0008, kd = 0.2
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

  // if (desired_v_x != prev_vel[0]) {
  //   cmd_vel[0] += copysign(linear_ramp, desired_v_x - prev_vel[0]);
  // } else {
  //   cmd_vel[0] += 0.0;
  // }

  // if (desired_v_y != prev_vel[1]) {
  //   cmd_vel[1] += copysign(linear_ramp, desired_v_y - prev_vel[1]);
  // } else {
  //   cmd_vel[1] += 0.0;
  // }

  // if (desired_v_z != prev_vel[2]) {
  //   cmd_vel[2] += copysign(linear_ramp, desired_v_z - prev_vel[2]);
  // } else {
  //   cmd_vel[2] += 0.0;
  // }

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

  // for (int vel_idx = 3; vel_idx <= 5; vel_idx++) {
  //   cmd_vel[vel_idx] = (std::abs(cmd_vel[vel_idx]) < angular_max)
  //                          ? cmd_vel[vel_idx]
  //                          : copysign(angular_max, cmd_vel[vel_idx]);
  // }

  // std::cout << "--------------------------INFO-----------------------------" << std::endl;
  // std::cout << "e_v_x:" << e_v_x << " e_v_y:" << e_v_y << " e_v_z:" << e_v_z << std::endl;
  // std::cout << "e_w_x:" << e_w_x << " e_w_y:" << e_w_y << " e_w_z:" << e_w_z << std::endl;
  // std::cout << "de_v_x:" << de_v_x << " de_v_y:" << de_v_y << " de_v_z:" << de_v_z << std::endl;
  // std::cout << "de_w_x:" << de_w_x << " de_w_y:" << de_w_y << " de_w_z:" << de_w_z << std::endl;
  // std::cout << "v_x:" << cmd_vel[0] << "\tv_y:" << cmd_vel[1] << "\tv_z:" << cmd_vel[2]
  //           << std::endl;
  // std::cout << "w_x:" << cmd_vel[3] << "\tw_y:" << cmd_vel[4] << "\tw_z:" << cmd_vel[5]
  //           << std::endl;
  // std::cout << "-----------------------------------------------------------" << std::endl;

  return cmd_vel;
}  // namespace franka_example_controllers

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;
  last_pose_ = initial_pose_;
  target_pose_ = {{initial_pose_[0], initial_pose_[1], initial_pose_[2], initial_pose_[4],
                   initial_pose_[5], initial_pose_[6], initial_pose_[8], initial_pose_[9],
                   initial_pose_[10], initial_pose_[12], initial_pose_[13], initial_pose_[14]}};
  elapsed_time_ = ros::Duration(0.0);
  current_time = 0.0;
  last_time = 0.0;
  isContact = false;
  last_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  cmd_vel = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  cmd_acc = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  last_cmd_acc = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  isContact_msg =
      nh_.subscribe("isContact", 1, &CartesianVelocityExampleController::isContact_callback, this);
  cmd_pos_msg = nh_.subscribe("franka_cmd_pos", 1,
                              &CartesianVelocityExampleController::cmd_pos_callback, this);
  cmd_vel_msg = nh_.subscribe("franka_cmd_vel", 1,
                              &CartesianVelocityExampleController::cmd_vel_callback, this);
  cmd_acc_msg = nh_.subscribe("franka_cmd_acc", 1,
                              &CartesianVelocityExampleController::cmd_acc_callback, this);
  // cmd_acc_msg =
  //     nh_.subscribe("cmd_js", 1, &CartesianVelocityExampleController::cmd_acc_callback, this);
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
  current_wrench = velocity_cartesian_handle_->getRobotState().K_F_ext_hat_K;
  double desired_Fz = 1.0;  // [N]

  std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  // std::array<double, 6> command = last_command;
  std::array<double, 12> curr_target = target_pose_;

  // recalculate translation under contact mode
  // if (isContact) {
  //   auto Pz = forceControl(curr_target, current_wrench, desired_Fz);
  //   curr_target[9] = Pz[0];
  //   curr_target[10] = Pz[1];
  //   curr_target[11] = Pz[2];
  // }

  // controls
  if (ctrl_mode == 'p') {
    /* if sending desired eef pose */
    command = calcNewVel(curr_target, current_pose_, last_pose_, last_command, period);
  } else if (ctrl_mode == 'v') {
    /* if sending desired eef velocity */
    command = cmd_vel;
  } else if (ctrl_mode == 'a') {
    /* if sending desired eef acceleration */
    for (size_t i = 0; i < 6; i++) {
      command[i] += cmd_acc[i];
    }
  }

  // safety check
  auto command2send = command;
  for (size_t i = 0; i < 6; i++) {
    if (std::abs(cmd_acc[i] - last_cmd_acc[i]) > 0.03) {
      ROS_WARN_STREAM("TOO LARGE JERK, COMMAND REJECTED!");
      command2send = last_command;
      break;
    }
    if (std::abs((command2send[i] - last_command[i]) * period.toSec()) > 0.01) {
      ROS_WARN_STREAM("TOO LARGE ACCELERATION, COMMAND REJECTED!");
      command2send = last_command;
      break;
    }
    if ((std::abs(current_wrench[0]) + std::abs(current_wrench[0]) + std::abs(current_wrench[0])) >
        30.0) {
      ROS_WARN_STREAM("TOO LARGE WRENCH, COMMAND REJECTED!");
      command2send = last_command;
      break;
    }
    if (std::abs(command2send[i]) > 0.06) {
      ROS_WARN_STREAM("TOO LARGE VELOCITY, COMMAND REJECTED!");
      command2send = last_command;
      break;
    }
  }
  velocity_cartesian_handle_->setCommand(command2send);

  last_cmd_acc = cmd_acc;
  last_command = command2send;
  last_pose_ = current_pose_;

  if (current_time - last_time >= 1.0) {  // report current status
    updateStatus();
    std::cout << "ctrl_mode: " << ctrl_mode << std::endl;
    last_time = current_time;
  }
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)

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
    for (size_t i = 0; i < q_start.size()-1; i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 5) {
        ROS_ERROR_STREAM(
            "CartesianVelocityExampleController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE;
  double current_campose[16];
  current_campose[0] = current_pose[0];
  current_campose[1] = current_pose[1];
  current_campose[2] = current_pose[2];
  current_campose[3] = 0.0;
  current_campose[4] = -current_pose[4];
  current_campose[5] = -current_pose[5];
  current_campose[6] = -current_pose[6];
  current_campose[7] = 0.0;
  current_campose[8] = -current_pose[8];
  current_campose[9] = -current_pose[9];
  current_campose[10] = -current_pose[10];
  current_campose[11] = 0.0;
  current_campose[12] = 0.16*current_pose[8]+current_pose[12]-0.035*current_pose[4];
  current_campose[13] = 0.16*current_pose[9]+current_pose[13]-0.035*current_pose[5];
  current_campose[14] = 0.16*current_pose[10]+current_pose[14]-0.035*current_pose[6];
  current_campose[15] = 1.0;

  double c_roll = atan2(current_campose[6],current_campose[10]);
  double c_yaw = atan2(current_campose[1],current_campose[0]);
  double c_pitch = atan2(-current_campose[2],cos(c_yaw)*current_campose[0]+sin(c_yaw)*current_campose[1]);
  double c_x = current_campose[12];
  double c_y = current_campose[13];
  double c_z = current_campose[14];
  preposes_.linear.x = c_x;
  preposes_.linear.y = c_y;
  preposes_.linear.z = c_z;
  preposes_.angular.x = c_roll;
  preposes_.angular.y = c_pitch;
  preposes_.angular.z = c_yaw;

  vels_.linear.x = 0.0;
  vels_.linear.y = 0.0;
  vels_.linear.z = 0.0;
  vels_.angular.x = 0.0;
  vels_.angular.y = 0.0;
  vels_.angular.z = 0.0;
  prevels_.linear.x = 0.0;
  prevels_.linear.y = 0.0;
  prevels_.linear.z = 0.0;
  prevels_.angular.x = 0.0;
  prevels_.angular.y = 0.0;
  prevels_.angular.z = 0.0;
  
  subpos_ = subpos_node_.subscribe("/cmd_pos", 1, &CartesianVelocityExampleController::POSCallback, this);
  subvel_ = subvel_node_.subscribe("/cmd_vel", 1, &CartesianVelocityExampleController::VELCallback, this);

}


void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  // ROS_ERROR_STREAM("==============");
  current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE;

  double dt = period.toSec();

  double current_campose[16];

  current_campose[0] = current_pose[0];
  current_campose[1] = current_pose[1];
  current_campose[2] = current_pose[2];
  current_campose[3] = 0.0;
  current_campose[4] = -current_pose[4];
  current_campose[5] = -current_pose[5];
  current_campose[6] = -current_pose[6];
  current_campose[7] = 0.0;
  current_campose[8] = -current_pose[8];
  current_campose[9] = -current_pose[9];
  current_campose[10] = -current_pose[10];
  current_campose[11] = 0.0;
  current_campose[12] = 0.16*current_pose[8]+current_pose[12]-0.035*current_pose[4];
  current_campose[13] = 0.16*current_pose[9]+current_pose[13]-0.035*current_pose[5];
  current_campose[14] = 0.16*current_pose[10]+current_pose[14]-0.035*current_pose[6];
  current_campose[15] = 1.0;

  double c_roll = atan2(current_campose[6],current_campose[10]);
  double c_yaw = atan2(current_campose[1],current_campose[0]);
  double c_pitch = atan2(-current_campose[2],std::sqrt(current_campose[6]*current_campose[6]+current_campose[10]*current_campose[10]));

  // if ((current_campose[2]!=1) && (current_campose[2]!=-1))
  // {
  //   double pitch1 = -asin(current_campose[2]);
  //   double pitch2 = 3.1415 - pitch1;
  //   double roll1 = atan2(current_campose[6]/cos(pitch1),current_campose[10]/cos(pitch1));
  //   double roll2 = atan2(current_campose[6]/cos(pitch2),current_campose[10]/cos(pitch2));
  //   double yaw1 = atan2(current_campose[1]/cos(pitch1),current_campose[0]/cos(pitch1));
  //   double yaw2 = atan2(current_campose[1]/cos(pitch2),current_campose[0]/cos(pitch2));
  //   if (std::abs(roll1 - goals_.angular.x) < std::abs(roll2 - goals_.angular.x))
  //     double c_roll = roll1;
  //   else
  //     double c_roll = roll2;
  
  //   if (std::abs(pitch1 - goals_.angular.y) < std::abs(pitch2 - goals_.angular.y))
  //     double c_pitch = pitch1;
  //   else
  //     double c_pitch = pitch2; 

  //   if (std::abs(yaw1 - goals_.angular.z) < std::abs(yaw2 - goals_.angular.z))
  //     double c_yaw = yaw1;
  //   else
  //     double c_yaw = yaw2; 
  // }
  // else
  // {
  //   double c_yaw = goals_.angular.z;
  //   if (current_campose[2]==-1)
  //   {
  //     double c_pitch = 3.1415/2;
  //     double c_roll = c_yaw + atan2(current_campose[4],current_campose[8]);
  //   }
  //   else if (current_campose[2]==1)
  //   {
  //     double c_pitch = -3.1415/2;
  //     double c_roll = c_yaw + atan2(-current_campose[4],-current_campose[8]);
  //   }
  // }
  
  double c_x = current_campose[12];
  double c_y = current_campose[13];
  double c_z = current_campose[14];

  // ROS_ERROR_STREAM("X:"<<c_x<<","<<c_y<<","<<c_z<<","<<c_roll<<","<<c_pitch<<","<<c_yaw);
  double e_x = goals_.linear.x - c_x;
  double e_y = goals_.linear.y - c_y;
  double e_z = goals_.linear.z - c_z;
  double e_roll = goals_.angular.x - c_roll;
  double e_pitch = goals_.angular.y - c_pitch;
  double e_yaw = goals_.angular.z - c_yaw;
  
  // ROS_ERROR_STREAM("pos: "<<c_x<<", "<<c_y<<", "<<c_z<<", "<<c_roll<<", "<<c_pitch<<", "<<c_yaw);

  // ROS_ERROR_STREAM("err: "<<e_x<<", "<<e_y<<", "<<e_z<<", "<<e_roll<<", "<<e_pitch<<", "<<e_yaw);

  double ed_x = Dvels_.linear.x - (c_x - preposes_.linear.x)/dt;
  double ed_y = Dvels_.linear.y - (c_y - preposes_.linear.y)/dt;
  double ed_z = Dvels_.linear.z - (c_z - preposes_.linear.z)/dt;
  double mov_roll = c_roll - preposes_.angular.x;
  double mov_pitch = c_pitch - preposes_.angular.y;
  double mov_yaw = c_yaw - preposes_.angular.z;

  double ed_roll = Dvels_.angular.x - (mov_roll)/dt;
  double ed_pitch = Dvels_.angular.y - (mov_pitch)/dt;
  double ed_yaw = Dvels_.angular.z - (mov_yaw)/dt;

  vels_.linear.x = 0.82 * e_x; // + 0.5 * ed_x;
  vels_.linear.y = 0.82 * e_y; // + 0.5 * ed_y;
  vels_.linear.z = 0.82 * e_z; // + 0.5 * ed_z;
  vels_.angular.x = 0.82 * e_roll; // + 0.5 * ed_roll;
  vels_.angular.y = 0.82 * e_pitch; // + 0.5 * ed_pitch;
  vels_.angular.z = 0.5 * e_yaw; // + 0.5 * ed_yaw;

  // ROS_ERROR_STREAM("vel: "<<vels_.linear.x<<", "<<vels_.linear.y<<", "<<vels_.linear.z<<","<<vels_.angular.x<<", "<<vels_.angular.y<<", "<<vels_.angular.z);

  preposes_.linear.x = c_x;
  preposes_.linear.y = c_y;
  preposes_.linear.z = c_z;
  preposes_.angular.x = c_roll;
  preposes_.angular.y = c_pitch;
  preposes_.angular.z = c_yaw;

  // Ramping Velocity
  double linear_rampstep = 0.0001;
  double angular_rampstep = 0.0001;

  //Linear X
  if(vels_.linear.x > prevels_.linear.x){
    prevels_.linear.x += linear_rampstep;  }
  //else if(vels_.linear.x == prevels_.linear.x){
    //prevels_.linear.x += 0.0;    }
  else{
    prevels_.linear.x -= linear_rampstep;  }
  // Linear Y
  if(vels_.linear.y > prevels_.linear.y){
    prevels_.linear.y += linear_rampstep;  }
  //else if(vels_.linear.y == prevels_.linear.y){
    //prevels_.linear.y += 0.0;    }
  else{
    prevels_.linear.y -= linear_rampstep;  }
  // Linear Z
  if(vels_.linear.z > prevels_.linear.z){
    prevels_.linear.z += linear_rampstep;  }
  //else if(vels_.linear.z == prevels_.linear.z){
    //prevels_.linear.z += 0.0;    }
  else{
    prevels_.linear.z -= linear_rampstep;  }
  // Angular X
  if(vels_.angular.x > prevels_.angular.x){
    prevels_.angular.x += angular_rampstep;  }
  //else if(vels_.angular.x == prevels_.angular.x){
    //prevels_.angular.x += 0.0;    }
  else{
    prevels_.angular.x -= angular_rampstep;  }
  // Angular Y
  if(vels_.angular.y > prevels_.angular.y){
    prevels_.angular.y += angular_rampstep;  }
  //else if(vels_.angular.y == prevels_.angular.y){
    //prevels_.angular.y += 0.0;    }
  else{
    prevels_.angular.y -= angular_rampstep;  }
  // Angular Z
  if(vels_.angular.z > prevels_.angular.z){
    prevels_.angular.z += angular_rampstep;  }
  //else if(vels_.angular.z == prevels_.angular.z){
    //prevels_.angular.z += 0.0;    }
  else{
    prevels_.angular.z -= angular_rampstep;  }

  std::array<double, 6> command = {{prevels_.linear.x, prevels_.linear.y, prevels_.linear.z, prevels_.angular.x, prevels_.angular.y, prevels_.angular.z}};
  // std::array<double, 6> command = {{prevels_.linear.x, prevels_.linear.y, prevels_.linear.z, 0, prevels_.angular.y, prevels_.angular.z}};

  if (c_yaw < - 1.8 || c_yaw > 1.8)
  {
    //std::cout<<prevels_.angular.x<<"\t"<<prevels_.angular.y<<"\t"<<prevels_.angular.z<<std::endl;
    // std::cout<<goals_.angular.x<<"\t"<<goals_.angular.y<<"\t"<<goals_.angular.z<<std::endl;
    // std::cout<<c_roll<<"\t\t"<<c_pitch<<"\t\t"<<c_yaw<<std::endl;
    // std::cout<<e_roll<<"\t"<<e_pitch<<"\t"<<e_yaw<<std::endl;
    /*
    if ((std::abs(e_x) < 0.1) && (std::abs(e_y) < 0.1) && (std::abs(e_z) < 0.1))
    {
      std::array<double, 6> command = {{0,0,0,0,0,0}};
      // std::cout<<"stopped"<<std::endl;
    }
    */
    // std::array<double, 6> command = {{0,0,0,0,0,0}};
    // std::cout<<"stopping ..."<<std::endl;
  }

  // ROS_ERROR_STREAM("cmd: "<<prevels_.linear.x<<", "<<prevels_.linear.y<<", "<<prevels_.linear.z<<"\n");

  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

// SG - 0725
void CartesianVelocityExampleController::POSCallback(const geometry_msgs::Twist &msg) {
   posflag_ = true;
   goals_ = msg;
   posflag_ = false;
}

void CartesianVelocityExampleController::VELCallback(const geometry_msgs::Twist &msg) {
   velflag_ = true;
   Dvels_ = msg;
   velflag_ = false;
}
// End SG 0725

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)

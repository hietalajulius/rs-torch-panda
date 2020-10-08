// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <controller_test/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include<thread>
#include<std_msgs/Int16.h>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include<std_msgs/String.h>
#include<Eigen/Eigen>
#include <iostream>
#include <math.h>

using namespace Eigen;

VectorXd calculate_a(double t_0, double t_f, VectorXd b) {
    MatrixXd qp(6,6);
    qp << 1, t_0, pow(t_0, 2), pow(t_0, 3), pow(t_0, 4), pow(t_0, 5),
        0, 1, 2*t_0, 3*pow(t_0, 2), 4*pow(t_0, 3), 5*pow(t_0, 4),
        0, 0, 2, 6*t_0, 12*pow(t_0, 2), 20*pow(t_0, 3),
        1, t_f, pow(t_f, 2), pow(t_f, 3), pow(t_f, 4), pow(t_f, 5),
        0, 1, 2*t_f, 3*pow(t_f, 2), 4*pow(t_f, 3), 5*pow(t_f, 4),
        0, 0, 2, 6*t_f, 12*pow(t_f, 2), 20*pow(t_f, 3);

    MatrixXd qp_inv(6,6);
    qp_inv << qp.inverse();
    VectorXd a(6);
    a << qp_inv*b;
    return a;

}

double position(VectorXd a, double t) {
    VectorXd t_vec(6);
    t_vec << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5);
    double q_t = t_vec.dot(a);
    return q_t;
}

namespace controller_test {



void callback(const std_msgs::Int16& msg) {
  std::cout << std::endl << "Current number" << msg.data << "\r" << std::endl;
}


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
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch controller_test move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  pub = node_handle.advertise<std_msgs::String>("/ready_robot", 10);
  sub = node_handle.subscribe("/positions", 1000, callback);

  double t_0 = 0.0;
  t_f = 0.1;
  double q_0 = 0.0;
  double q_f = 0.01;
  double v_0 = 0.0; 
  double v_f = 0.0;
  double alpha_0 = 0.0;
  double alpha_f = 0.0;
  b_vec << q_0, v_0, alpha_0, q_f, v_f, alpha_f;
  a_vec << calculate_a(t_0, t_f, b_vec);


  return true;
}


void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  actions = 0.0;
  delta_x = 0;
}

void CartesianPoseExampleController::update_current_number(const std_msgs::Int16& msg) {
  current_number = msg.data;
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {


  std_msgs::String msg;
  msg.data = std::to_string(elapsed_time_.toSec());
  pub.publish(msg);
  
  elapsed_time_ += period;
  actions += 1;

  if (elapsed_time_.toSec() < t_f) {
    delta_x = position(a_vec, elapsed_time_.toSec());
    std::cout <<  "Pos: " << delta_x << " time: " << elapsed_time_  << "\r" <<std::endl;
  }
  

  /*
  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  */

  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] += delta_x;
  // new_pose[14] += delta_z;
  cartesian_pose_handle_->setCommand(new_pose);
  // std::cout <<  "Freq: " << actions/elapsed_time_.toSec() << " timez: " << elapsed_time_ << "\r" <<std::endl;
  // std::cout << std::endl << "Current number" << current_number << "\r" << std::endl;
}

}  // namespace controller_test

PLUGINLIB_EXPORT_CLASS(controller_test::CartesianPoseExampleController,
                       controller_interface::ControllerBase)

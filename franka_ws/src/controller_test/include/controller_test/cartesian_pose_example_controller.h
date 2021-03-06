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
#include<ros/ros.h>
#include<std_msgs/Int16.h>
#include<Eigen/Eigen>
#include <iostream>
#include <math.h>

using namespace Eigen;

namespace controller_test {

class CartesianPoseExampleController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void update_current_number(const std_msgs::Int16& msg);
  

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};
  double actions;
  ros::Publisher pub;
  ros::Subscriber sub;
  double current_number;
  Eigen::VectorXd b_vec = Eigen::VectorXd(6);
  Eigen::VectorXd a_vec = Eigen::VectorXd(6);
  double delta_x;
  double t_f;
};

}  // namespace controller_test

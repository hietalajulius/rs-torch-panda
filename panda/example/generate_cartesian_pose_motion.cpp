// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <fstream>
#include <string>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <cassert>
#include <sstream>

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */


using namespace Eigen;
using namespace std;

#define MAXBUFSIZE  ((int) 1e4)

void readMatrix(const char *filename)
    {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
        {
        string line;
        getline(infile, line);
        std::cout << "Line: " << line << "\n" << std::endl;
        
        }

    infile.close();

    rows--;


    };

int main(int argc, char** argv) {
  //readMatrix("/home/julius/robotics/cloth-manipulation/experiments/diagonal_ee_positions.csv");
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{-0.102, -0.116, -0.364, -2.68, -0.08, 2.58, 0.396}};
    //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;
    double prev_delta = 0.0;
    int steps_taken = 0;
    double evolving_delta = 0.0;
    ifstream infile;
    infile.open("/home/julius/robotics/cloth-manipulation/experiments/diagonal_ee_positions.txt");
    robot.control([&time, &initial_pose, &prev_delta, &steps_taken, &evolving_delta, &infile](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      steps_taken += 1;

      std::cout << "Robot state x: " << robot_state.O_T_EE_c[12] << "\n" << std::endl;
      std::cout << "Robot state y: " << robot_state.O_T_EE_c[13] << "\n" << std::endl;
      std::cout << "Robot state z: " << robot_state.O_T_EE_c[14] << "\n" << std::endl;
      std::cout << "Time: " << time << "\n" << std::endl;

      std::cout << "Period: " << period.toSec() << "\n" << std::endl;
      string line;
      getline(infile, line);
      std::cout << "Line: " << line << "\n" << std::endl;

      std::vector<float> v;

      // Build an istream that holds the input string
      std::istringstream iss(line);

      // Iterate over the istream, using >> to grab floats
      // and push_back to store them in the vector
      std::copy(std::istream_iterator<float>(iss),
            std::istream_iterator<float>(),
            std::back_inserter(v));

      // Put the result on standard out
      std::cout << "d x: " << v[0] << "\n" << std::endl;
      std::cout << "d y: " << v[1] << "\n" << std::endl;
      std::cout << "d z: " << v[2] << "\n" << std::endl;

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      
      constexpr double kRadius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = kRadius * std::sin(angle);
      double delta_z = kRadius * (std::cos(angle) - 1);

      std::cout << "True delta: " << delta_x - prev_delta << "\n" << std::endl;

      prev_delta = delta_x;


      

      
      //std::array<double, 16> new_pose = robot_state.O_T_EE_d;
      std::array<double, 16> new_pose = initial_pose;

      //new_pose[12] += delta_x;
      new_pose[12] += v[0]/1000;
      new_pose[13] += v[1]/1000;
      new_pose[14] += v[2]/1000;
      
      /*
      if (steps_taken < 40) {
        new_pose[12] += 0.00000001*steps_taken;
      } */

      //evolving_delta += 0.0001;
      //new_pose[12] += evolving_delta * 0.001;
      
      // new_pose[14] += delta_z;

      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

cmake_minimum_required(VERSION 3.0)
project(quintic)
add_executable(quintic src/quintic_polynomial.cpp)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (quintic Eigen3::Eigen)
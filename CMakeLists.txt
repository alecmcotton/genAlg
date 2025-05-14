cmake_minimum_required(VERSION 3.10)
project(RobotKinematics)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)

add_executable(main main.cpp kinematics.cpp)
target_include_directories(main PRIVATE ${EIGEN3_INCLUDE_DIR})




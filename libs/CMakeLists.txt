cmake_minimum_required(VERSION 3.0.2)
project(libs)

find_package(catkin REQUIRED)


catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


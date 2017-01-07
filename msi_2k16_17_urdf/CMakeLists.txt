cmake_minimum_required(VERSION 2.8.3)
project(msi_2k16_17_urdf)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)


catkin_add_env_hooks(rover_description
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_package()

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(Odometry src/Odometry.cpp)
target_link_libraries(Odometry ${catkin_LIBRARIES})

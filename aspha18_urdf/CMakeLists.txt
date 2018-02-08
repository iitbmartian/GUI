cmake_minimum_required(VERSION 2.8.3)
project(aspha18_urdf)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)


catkin_package()

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(Odometry_new src/Odometry_new.cpp)
target_link_libraries(Odometry_new ${catkin_LIBRARIES}) 

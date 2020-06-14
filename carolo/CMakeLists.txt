cmake_minimum_required(VERSION 2.8.3)
project(carolo)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  std_msgs
  message_generation
  gazebo_ros 
)

find_package(gazebo REQUIRED) # Depend on system install of Gazebo
find_package(xacro REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DriveCommand_itmoves.msg
  DriveState_itmoves.msg
  RemoteControl_itmoves.msg
)

#Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

add_library(itmoves_car_plugin SHARED plugins/itmoves_car_plugin.cc)
target_link_libraries(itmoves_car_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(itmoves_car_plugin ${catkin_EXPORTED_TARGETS})

#install(TARGETS itmoves_car_plugin
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




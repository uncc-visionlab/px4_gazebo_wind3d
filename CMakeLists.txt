cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

set(PX4_GAZEBO_CLASSIC_SITL_ROOT /home2/arwillis/uncc_wind_control/third_party/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/)

if(NOT DEFINED NO_ROS)
  message(STATUS "NO_ROS variable not provided, setting to FALSE.")
  set(NO_ROS FALSE)
endif()

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

# CMake build type (Debug Release RelWithDebInfo MinSizeRel)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

project(px4_gazebo_wind3d VERSION 1.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
#find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)

# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #
if (NOT NO_ROS)
  find_package(catkin REQUIRED COMPONENTS
    gazebo_plugins
    geometry_msgs
#    mav_msgs
    message_generation
    roscpp
    std_msgs
    tf
  )
endif()

add_message_files(
  FILES
  Anemometer.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

# Note: If using catkin, Python 2 is found since it points
# to the Python libs installed with the ROS distro
if (NOT CATKIN_DEVEL_PREFIX)
	find_package(PythonInterp 3 REQUIRED)
else()
	find_package(roscpp REQUIRED)
	message(STATUS "${roscpp_version}")
	if(${roscpp_VERSION}  VERSION_LESS "1.15.0")
		find_package(PythonInterp REQUIRED)
	else()
		find_package(PythonInterp 3 REQUIRED)
	endif()
endif()
#find_package(OpenCV REQUIRED)
#find_package(TinyXML REQUIRED)

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
else()
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

# for ROS2 subscribers and publishers
if (BUILD_ROS2_PLUGINS)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
endif()


# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

add_compile_options(-Wno-deprecated-declarations -Wno-address-of-packed-member)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  )

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

#--------------------------#
# Generation of SDF models #
#--------------------------#

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(enable_wind "false")
set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
# set the vision estimation to be sent if set by the CMake option SEND_VISION_ESTIMATION_DATA
set(send_vision_estimation "false")
if (SEND_VISION_ESTIMATION_DATA)
  set(send_vision_estimation "true")
endif()

# if SEND_ODOMETRY_DATA option is set, then full odometry data is sent instead of
# only the visual pose estimate
set(send_odometry "false")
if (SEND_ODOMETRY_DATA)
  set(send_odometry "true")
  set(send_vision_estimation "false")
endif()

#--------------------#
# Message Generation #
#--------------------#
include_directories(${GAZEBO_INCLUDE_DIRS} ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/include)
set(mav_msgs
  ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/CommandMotorSpeed.proto
  ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/MotorSpeed.proto
)
set(nav_msgs ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Odometry.proto)
set(physics_msgs
   ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Force.proto
   ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Wind.proto
)
set(std_msgs ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Int32.proto)
set(sensor_msgs
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Airspeed.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Imu.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/IRLock.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/TargetRelative.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Float.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Groundtruth.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Range.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/SITLGps.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/OpticalFlow.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/MagneticField.proto
    ${PX4_GAZEBO_CLASSIC_SITL_ROOT}/msgs/Pressure.proto
  )
set(wind3d_msgs
    msgs/ConnectGazeboToRosTopic.proto
    msgs/ConnectRosToGazeboTopic.proto
    msgs/Header.proto
    msgs/Anemometer.proto
    msgs/WindSpeed.proto
    msgs/WindServerRegistration.proto
  )
PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})
#PROTOBUF_GENERATE_CPP(NAV_PROTO_SRCS NAV_PROTO_HDRS ${nav_msgs})
PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS ${physics_msgs})
#PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})
PROTOBUF_GENERATE_CPP(WIND3D_PROTO_SRCS WIND3D_PROTO_HDRS ${wind3d_msgs})


# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

if (NOT NO_ROS)

  if(${gazebo_dev_FOUND})
    catkin_package(
      INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#      LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
#      CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp gazebo_plugins rotors_comm rotors_control std_srvs tf
#      DEPENDS Eigen octomap OpenCV
      #CFG_EXTRAS rotors_gazebo_plugins.cmake
    )
    set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${gazebo_dev_LIBRARIES})
  else()
    catkin_package(
      INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#      LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
#      CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp gazebo_plugins rotors_comm rotors_control std_srvs tf
#      DEPENDS Eigen octomap OpenCV
      #CFG_EXTRAS rotors_gazebo_plugins.cmake
    )
    set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  endif()
endif()

# =============================================================================================== #
# ========================================= USER LIBRARIES ====================================== #
# =============================================================================================== #

add_library(mav_msgs SHARED ${MAV_PROTO_SRCS})
#add_library(nav_msgs SHARED ${NAV_PROTO_SRCS})
add_library(physics_msgs SHARED ${PHY_PROTO_SRCS})
#add_library(std_msgs SHARED ${STD_PROTO_SRCS})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})
add_library(wind3d_msgs SHARED ${WIND3D_PROTO_SRCS})

#---------#
# Plugins #
#---------#

#link_libraries(mav_msgs nav_msgs std_msgs sensor_msgs)
link_libraries(mav_msgs physics_msgs sensor_msgs wind3d_msgs)

add_library(gazebo_wind3d_world_plugin SHARED src/gazebo_wind3d_world_plugin.cpp)

add_library(gazebo_dynamic_wind_plugin SHARED src/gazebo_dynamic_wind_plugin.cpp)
add_library(gazebo_anemometer_plugin SHARED src/gazebo_anemometer_plugin.cpp)

add_library(gazebo_wind3d_motor_model_plugin SHARED src/gazebo_wind3d_motor_model_plugin.cpp)

set(plugins
  gazebo_wind3d_world_plugin
  gazebo_dynamic_wind_plugin
  gazebo_anemometer_plugin
  gazebo_wind3d_motor_model_plugin
  )

foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${TinyXML_LIBRARIES})
endforeach()

foreach(plugin ${plugins})
#  add_dependencies(${plugin} mav_msgs nav_msgs std_msgs sensor_msgs)
  add_dependencies(${plugin} mav_msgs physics_msgs)
endforeach()

# Configure the setup script
if (catkin_FOUND)
  catkin_add_env_hooks(50_sitl_gazebo_setup
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    SHELLS sh)
endif()

#===================================== ROS INTERFACE PLUGIN =====================================//
# This entire plugin is only built if ROS is a dependency
if (NOT NO_ROS)
  add_library(gazebo_ros_interface_plugin SHARED src/gazebo_ros_interface_plugin.cpp)
  target_link_libraries(gazebo_ros_interface_plugin ${target_linking_LIBRARIES} )
  add_dependencies(gazebo_ros_interface_plugin ${catkin_EXPORTED_TARGETS})
  list(APPEND targets_to_install gazebo_ros_interface_plugin)
endif()

#############
## Install ##
#############

#set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
#set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
#set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
#
#file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
#file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)
#
#file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
#file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)
#
#install(TARGETS ${plugins} mav_msgs nav_msgs std_msgs sensor_msgs DESTINATION ${PLUGIN_PATH})
#install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
#install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)
#
#configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})
#
#install(FILES ${PROJECT_SOURCE_DIR}/package.xml DESTINATION ${RESOURCE_PATH})

#############
## Testing ##
#############

# TODO

###############
## Packaging ##
###############

#set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
#set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
#set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
#set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
#set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
#set(DEBIAN_PACKAGE_DEPENDS "")
#set(RPM_PACKAGE_DEPENDS "")
#
#set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
#set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
#set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
#set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for px4 sitl.")
#
#set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
#set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for px4 sitl.")
#set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
#set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
#
#include(CPack)

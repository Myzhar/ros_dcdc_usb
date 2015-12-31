cmake_minimum_required(VERSION 2.8.7)

project(ros_dcdc_usb)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# LIBRARIES
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# LibUSB-0.1
find_package(libusb REQUIRED)
include_directories (${LIBUSB_INCLUDE_DIRS})
#########################################################

#########################################################
# INCLUDES
set(INC_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/include)

include_directories(${INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE SRC *.cpp *.c)
# HEADERS
file(GLOB_RECURSE HDR *.h)
#########################################################

#########################################################
#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  diagnostic_msgs
  diagnostic_updater
)

add_message_files(
  FILES
  DCDCStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
    diagnostic_msgs
    diagnostic_updater
)

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    dcdc_usb_node
    ${HDR}
    ${SRC}
)

target_link_libraries(
    dcdc_usb_node
    ${catkin_LIBRARIES}
    ${LIBUSB_LIBRARIES}
)
#########################################################

cmake_minimum_required(VERSION 3.5)
project(robot_state_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)
include_directories(${INC_DIR})
link_directories(${LINK_DIR})


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
include(GNUInstallDirs)
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

find_package(robot_interface REQUIRED)

add_executable(state_publish src/robot_state_publisher.cpp)
ament_target_dependencies(state_publish rclcpp std_msgs robot_interface)
target_link_libraries(state_publish iceoryx_posh::iceoryx_posh)

install(TARGETS
        state_publish
        DESTINATION lib/${PROJECT_NAME})

ament_package()
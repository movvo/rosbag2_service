cmake_minimum_required(VERSION 3.5)
project(rosbag2_service)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lgtest -lgmock)
endif()

#=====================================
#             Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression_zstd REQUIRED)
find_package(rosbag2_service_msg)
#=====================================
#          Build and Add executables
#=====================================

include_directories(
  include
#  test/include
)

add_library(${PROJECT_NAME}_lib
  "src/rosbag2_service.cpp"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp
                          rclcpp_components
                          std_srvs
                          std_msgs
                          rosbag2_cpp
                          rosbag2_compression_zstd
                          rosbag2_service_msg
                        )

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          rclcpp_components
                        )

#=====================================
#             Install
#=====================================
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})   

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)                   

#=====================================
#             Test
#=====================================

# if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_cmake_gmock REQUIRED)
  # ament_add_gmock(test_usb_mock test/src/test_usb_mock.cpp)
  # ament_target_dependencies(test_usb_mock
  # "rclcpp"
  # "ageve_interfaces"
  # "ageve_utils"
  # )
  # target_link_libraries(test_usb_mock ${PROJECT_NAME}_lib)
  # ament_add_gmock(test_usb_streamer test/src/test_usb_streamer.cpp)
  # ament_target_dependencies(test_usb_streamer
  #   "rclcpp"
  #   "ageve_interfaces"
  #   "ageve_utils"
  # )
  # target_link_libraries(test_usb_streamer ${PROJECT_NAME}_lib)
  # endif()


ament_package()

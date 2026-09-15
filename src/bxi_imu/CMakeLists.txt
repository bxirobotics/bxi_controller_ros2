cmake_minimum_required(VERSION 3.8)
project(bxi_imu)

# The bundled decoder is vendor code. Keep it inside this package without
# applying the controller repository's source-style checks to it.
set(AMENT_LINT_AUTO_FILE_EXCLUDE
  "third_party/hipnuc/include/*.h"
  "third_party/hipnuc/src/*.c"
  "src/backends/hipnuc_backend.cpp"
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(hipnuc_decoder STATIC
  third_party/hipnuc/src/canopen_parser.c
  third_party/hipnuc/src/hipnuc_can_common.c
  third_party/hipnuc/src/hipnuc_dec.c
  third_party/hipnuc/src/hipnuc_j1939_parser.c
  third_party/hipnuc/src/nmea_dec.c
)
target_include_directories(hipnuc_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/hipnuc/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(imu_node
  src/imu_node.cpp
  src/backend_factory.cpp
  src/backends/hipnuc_backend.cpp
)
target_include_directories(imu_node PRIVATE include)
ament_target_dependencies(imu_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(imu_node hipnuc_decoder)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

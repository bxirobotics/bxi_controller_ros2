cmake_minimum_required(VERSION 3.8)
project(bxi_imu)

# The bundled decoder is vendor code. Keep it inside this package without
# applying the controller repository's source-style checks to it.
set(AMENT_LINT_AUTO_FILE_EXCLUDE
  "modules/hipnuc/*.hpp"
  "modules/hipnuc/*.cpp"
  "modules/hipnuc/lib/**/*.h"
  "modules/hipnuc/lib/**/*.c"
  "modules/yesense/*.h"
  "modules/yesense/*.cpp"
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
  modules/hipnuc/lib/hipnuc_vendor/src/canopen_parser.c
  modules/hipnuc/lib/hipnuc_vendor/src/hipnuc_can_common.c
  modules/hipnuc/lib/hipnuc_vendor/src/hipnuc_dec.c
  modules/hipnuc/lib/hipnuc_vendor/src/hipnuc_j1939_parser.c
  modules/hipnuc/lib/hipnuc_vendor/src/nmea_dec.c
)
target_include_directories(hipnuc_decoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modules/hipnuc/lib/hipnuc_vendor/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(imu_node
  src/imu_node.cpp
  src/backend_factory.cpp
  modules/hipnuc/hipnuc_backend.cpp
  modules/yesense/yesense_backend.cpp
  modules/yesense/lib/yesense_decoder.cpp
  modules/yesense/lib/yesense_std_out_decoder.cpp
)
target_include_directories(imu_node PRIVATE
  include
  modules/hipnuc
  modules/yesense
  modules/hipnuc/lib/hipnuc_vendor/include
  modules/yesense/lib
)
ament_target_dependencies(imu_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(imu_node hipnuc_decoder)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY modules DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

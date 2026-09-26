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

add_executable(imu_node
  src/imu_node.cpp
  src/backend_factory.cpp
)
target_include_directories(imu_node PRIVATE
  include
)
ament_target_dependencies(imu_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(imu_node ${CMAKE_DL_LIBS})

# Every module is a self-contained shared-library plugin. Adding a new module
# directory with backend source files automatically adds it to the build.
file(GLOB MODULE_DIRECTORIES CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "modules/*")
foreach(MODULE_DIRECTORY IN LISTS MODULE_DIRECTORIES)
  get_filename_component(MODULE_NAME "${MODULE_DIRECTORY}" NAME)
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}" AND
    (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/backend.cpp" OR
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/${MODULE_NAME}_backend.cpp"))
    string(REGEX REPLACE "[^A-Za-z0-9_]" "_" MODULE_TARGET_SUFFIX "${MODULE_NAME}")
    file(GLOB_RECURSE MODULE_SOURCES CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/*.c")
    add_library("bxi_imu_module_${MODULE_TARGET_SUFFIX}" SHARED ${MODULE_SOURCES})
    target_include_directories("bxi_imu_module_${MODULE_TARGET_SUFFIX}" PRIVATE
      include
      "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}"
      "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/lib"
      "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_DIRECTORY}/lib/hipnuc_vendor/include")
    ament_target_dependencies("bxi_imu_module_${MODULE_TARGET_SUFFIX}" rclcpp sensor_msgs geometry_msgs)
    set_target_properties("bxi_imu_module_${MODULE_TARGET_SUFFIX}" PROPERTIES
      OUTPUT_NAME "bxi_imu_${MODULE_NAME}"
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/modules")
    install(TARGETS "bxi_imu_module_${MODULE_TARGET_SUFFIX}"
      LIBRARY DESTINATION lib/${PROJECT_NAME}/modules)
  endif()
endforeach()

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY modules DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

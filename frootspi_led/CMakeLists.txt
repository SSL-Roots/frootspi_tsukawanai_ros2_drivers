cmake_minimum_required(VERSION 3.5)
project(frootspi_led)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(led_driver_component SHARED
  src/led_driver_component.cpp)
target_compile_definitions(led_driver_component
  PRIVATE "FROOTSPI_LED_BUILDING_DLL")
ament_target_dependencies(led_driver_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs)
target_link_libraries(led_driver_component pigpiod_if2)
rclcpp_components_register_nodes(led_driver_component "frootspi_led::Driver")

add_executable(led_driver_node src/led_driver.cpp)
target_link_libraries(led_driver_node led_driver_component)
ament_target_dependencies(led_driver_node
  rclcpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(std_msgs)

ament_export_include_directories(include)
ament_export_libraries(led_driver_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  led_driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  led_driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

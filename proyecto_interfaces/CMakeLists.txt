cmake_minimum_required(VERSION 3.8)
project(proyecto_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/String.msg"
  "msg/Int32.msg"
  "msg/Bool.msg"
  "msg/Banner.msg"
  "srv/StartManipulationTest.srv"
  "srv/StartNavigationTest.srv"
  "srv/StartPerceptionTest.srv"
)

# Install Python executables
install(PROGRAMS
  src/serial_connection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/serial_connection_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python executables
install(PROGRAMS
  src/perception_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/manipulation_test.py
  DESTINATION lib/${PROJECT_NAME}
)

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop ${dependencies})
target_link_libraries(teleop "${cpp_typesupport_target}")

target_include_directories(teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
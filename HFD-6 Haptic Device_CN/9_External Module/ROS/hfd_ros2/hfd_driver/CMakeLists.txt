cmake_minimum_required(VERSION 3.8)
project(hfd_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hfd_msgs REQUIRED)
find_package(xjcsensor_msgs REQUIRED)

# Add include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Add library directories
link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib/${PROJECT_NAME}
  ${catkin_LIB_DIRS}
)

# Add executables
add_executable(hfd_interface
  src/hfd_interface.cpp
)

# Add libraries for executables
target_link_libraries(hfd_interface
  ${PROJECT_SOURCE_DIR}/lib/libHFD_API.so
  ${cpp_typesupport_target}
)

# Add dependencies for executables
ament_target_dependencies(hfd_interface
  rcl_interfaces
  rclcpp
  rosidl_default_runtime
  std_msgs
  hfd_msgs
  xjcsensor_msgs
)

# Testing package
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

# Install executables
install(TARGETS
  hfd_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(FILES
  ${PROJECT_SOURCE_DIR}/lib/libHFD_API.so
  DESTINATION lib
)

# Install resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()

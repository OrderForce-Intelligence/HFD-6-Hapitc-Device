cmake_minimum_required(VERSION 3.0.2)
project(HFD_Driver)

# 找到必要的catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)

# 添加自定义消息文件
add_message_files(
  FILES
  xd.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 链接库文件目录
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(hfd_driver src/hfd_driver.cpp)

# 指定依赖关系
add_dependencies(hfd_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库文件
target_link_libraries(hfd_driver
  ${catkin_LIBRARIES}
  HFD_API  # 链接库的名字，不带前缀和后缀
)


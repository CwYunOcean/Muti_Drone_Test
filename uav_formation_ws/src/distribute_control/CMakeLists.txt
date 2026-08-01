cmake_minimum_required(VERSION 3.8)
project(distribute_control)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# 1. 查找依赖
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 为了简洁，定义共用的曲线源文件变量
set(CURVE_SOURCES
  src/curves/ellipse_curve.cpp
  src/curves/eight_curve.cpp
  src/curves/squircle_curve.cpp
  src/curves/bspline_curve.cpp
  src/curves/eight_curve_2.cpp
)

# ============================================
# 2. 定义 Visualizer 节点 (可视化)
# ============================================
add_executable(visualizer_node
  src/nodes/visualizer_node.cpp
  ${CURVE_SOURCES}
)

add_executable(visualizer_set_node
  src/nodes/visualizer_set_node.cpp
  ${CURVE_SOURCES}
)

target_include_directories(visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(visualizer_set_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# [修复] 显式链接 Eigen 目标，自动包含头文件
target_link_libraries(visualizer_node Eigen3::Eigen)
target_link_libraries(visualizer_set_node Eigen3::Eigen)

ament_target_dependencies(visualizer_node
  rclcpp px4_msgs std_msgs visualization_msgs nav_msgs geometry_msgs
)
ament_target_dependencies(visualizer_set_node
  rclcpp px4_msgs std_msgs visualization_msgs nav_msgs geometry_msgs
)

# ============================================
# 3. 定义 Offboard 节点 (控制核心)
# ============================================
add_executable(offboard_node
  src/nodes/offboard_node.cpp
  src/core/controller.cpp
  src/core/topology.cpp
  src/core/orca_barrier.cpp
  ${CURVE_SOURCES}
)

target_include_directories(offboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# [修复] 显式链接 Eigen 目标，自动包含头文件
target_link_libraries(offboard_node Eigen3::Eigen)

ament_target_dependencies(offboard_node
  rclcpp px4_msgs std_msgs visualization_msgs nav_msgs geometry_msgs
  # Eigen3 <-- 从这里移除，因为已经在上面 target_link_libraries 链接了
)


# ============================================
# 3. 定义 Offboard 节点 (控制核心)
# ============================================
add_executable(offboard_node_2
  src/nodes/offboard_node_2.cpp
  src/core/topology.cpp
  src/core/orca_barrier.cpp
  src/core/controller_2.cpp
  ${CURVE_SOURCES}
)

target_include_directories(offboard_node_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# [修复] 显式链接 Eigen 目标，自动包含头文件
target_link_libraries(offboard_node_2 Eigen3::Eigen)

ament_target_dependencies(offboard_node_2
  rclcpp px4_msgs std_msgs visualization_msgs nav_msgs geometry_msgs
  # Eigen3 <-- 从这里移除，因为已经在上面 target_link_libraries 链接了
)

# ============================================
# 4. 安装规则
# ============================================

# 安装可执行文件
install(TARGETS visualizer_node offboard_node visualizer_set_node offboard_node_2
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置(params.yaml) 和 启动脚本
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# ============================================
# 5. 测试与打包
# ============================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_world_transform test/test_world_transform.cpp)
  target_include_directories(test_world_transform PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_world_transform Eigen3::Eigen)

  ament_add_pytest_test(
    test_topic_contract
    test/test_topic_contract.py
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
  )
endif()

ament_package()

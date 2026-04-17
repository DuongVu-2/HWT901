cmake_minimum_required(VERSION 3.8)
project(hwt901)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(tf REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_library(hwt901_sensor
  src/hwt901.cpp
)

target_include_directories(hwt901_sensor PUBLIC include)
target_link_libraries(hwt901_sensor ${Boost_LIBRARIES})
add_executable(imu_node src/hwt901_node.cpp)
target_include_directories(imu_node PRIVATE include)
ament_target_dependencies(imu_node 
  rclcpp 
  sensor_msgs 
  tf2
)
target_link_libraries(imu_node hwt901_sensor)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Cài đặt thư mục launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Nếu bạn có thư mục urdf, cũng nên cài đặt nó luôn
install(DIRECTORY
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

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

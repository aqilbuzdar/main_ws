cmake_minimum_required(VERSION 3.8)
project(dodo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Launch
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# URDF
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# Meshes
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY cartographer DESTINATION share/${PROJECT_NAME})

# Config (YAMLs + RViz)
install(
  FILES
    config/dodo.rviz
    config/slam.yaml
    config/ekf.yaml
    config/oak_d.yaml
    config/imu.rviz
    config/nav2_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Python nodes
install(PROGRAMS
  scripts/serial_node.py
  scripts/oak_imu.py
  scripts/main_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

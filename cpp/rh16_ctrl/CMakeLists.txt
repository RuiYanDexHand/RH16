cmake_minimum_required(VERSION 3.5)
project(rh16_ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-variable -Wno-cpp )
endif()

include_directories(include lib)

find_library(RUIYAN_SRC_LIB
    NAMES Ryhand64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
)

if (NOT RUIYAN_SRC_LIB)
    message(FATAL_ERROR "Ryhand64 library not found in rh16_ctrl/lib!")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rh16_cmd REQUIRED)
find_package(rh16_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(rh_ctrl src/ryhandlib_port.c src/can_socket.c src/rh16_ctrl.cpp )
target_include_directories(rh_ctrl PUBLIC include lib )
target_link_libraries(rh_ctrl
  ${RUIYAN_SRC_LIB}
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(rh_ctrl rclcpp rh16_cmd rh16_msg pinocchio)

add_executable(rh_test src/rh16_test.cpp )
target_include_directories(rh_test PUBLIC include )
ament_target_dependencies(rh_test rclcpp rh16_cmd)

add_executable(rh16_kinematics src/rh16_kinematics.cpp)
target_link_libraries(rh16_kinematics
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(rh16_kinematics sensor_msgs rclcpp pinocchio)

install(TARGETS
  rh_ctrl rh_test rh16_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# --- 关键修复 ---
# 使用更直接的方式安装所有.so文件到 install/lib/
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(pick_drop_node)

# ===============================
# 기본 설정
# ===============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# 의존성 패키지
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# ===============================
# Inbound Node
# ===============================
add_executable(forkfit_inbound_state_node
  src/ForkFit_Inbound_State_Node.cpp
)

ament_target_dependencies(forkfit_inbound_state_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# ===============================
# Outbound Node
# ===============================
add_executable(forkfit_outbound_state_node
  src/ForkFit_Outbound_State_Node.cpp
)

ament_target_dependencies(forkfit_outbound_state_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# ===============================
# Install
# ===============================
install(TARGETS
  forkfit_inbound_state_node
  forkfit_outbound_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ===============================
# Ament 패키지 선언
# ===============================
ament_package()

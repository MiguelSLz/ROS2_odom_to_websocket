cmake_minimum_required(VERSION 3.8)
project(ros2_unreal_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Encontra pacotes ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Encontra Boost e Threads
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

# Inclui diretórios
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  /usr/include/jsoncpp
)

# Adiciona o executável
add_executable(${PROJECT_NAME} src/ros2_websocket_bridge.cpp)

# Linka bibliotecas
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  jsoncpp
  Threads::Threads
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
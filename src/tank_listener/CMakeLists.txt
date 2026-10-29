cmake_minimum_required(VERSION 3.8)

project(tank_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(water_tank_interface REQUIRED)

add_executable(listener
    src/tank_pubsub.cpp
    src/control_algorithm.cpp
)

target_include_directories(listener PRIVATE
    include
)

ament_target_dependencies(listener
    rclcpp
    water_tank_interface
)

install(TARGETS
    listener
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
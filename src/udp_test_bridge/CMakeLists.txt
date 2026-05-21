cmake_minimum_required(VERSION 3.10)
project(udp_test_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(udp_test_receiver src/udp_test_receiver.cpp)
ament_target_dependencies(udp_test_receiver
    rclcpp
)

install(TARGETS udp_test_receiver
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

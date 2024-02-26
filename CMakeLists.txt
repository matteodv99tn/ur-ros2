cmake_minimum_required(VERSION 3.8)
project(magician_ur)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)


set(ROS2_REQUIRED_COMPONENTS
    rclcpp
    tf2_ros
    sensor_msgs
    geometry_msgs
    )
foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()


# include_directories(${CMAKE_CURRENT_LIST_DIR}/include)


add_executable(read_data
    src/read_data.cpp
    )
target_link_libraries(read_data
    Eigen3::Eigen
    )
ament_target_dependencies(read_data
    ${ROS2_REQUIRED_COMPONENTS}
    )

#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#
install(
    DIRECTORY
    launch
    config
    rviz
    urdf
    resources
    DESTINATION share/${PROJECT_NAME}
    )

install(
    TARGETS read_data
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(magician_ur)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


set(ROS2_REQUIRED_COMPONENTS
    tf2_ros
    )
foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()


include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

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
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()

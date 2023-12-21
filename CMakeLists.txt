cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# Define the source files for each executable
set(CMD_VEL_PUBLISHER_SOURCE src/cmd_vel_publisher_node.cpp)
set(HERO_CONTROL_SOURCE src/hero.control.cpp)
set(PID_CONTROLLER_SOURCE src/pid_controller_node.cpp)

# Add the executables
add_executable(cmd_vel_publisher_node ${CMD_VEL_PUBLISHER_SOURCE})
add_executable(hero_control ${HERO_CONTROL_SOURCE})
add_executable(pid_controller_node ${PID_CONTROLLER_SOURCE})
add_executable(inverse_kinematics_node src/inverse_kinematics.cpp)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
)


catkin_package(
        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        pluginlib
        DEPENDS
        INCLUDE_DIRS include
)




include_directories(
        ${catkin_INCLUDE_DIRS}

)


target_link_libraries(cmd_vel_publisher_node ${catkin_LIBRARIES})
target_link_libraries(hero_control ${catkin_LIBRARIES})
target_link_libraries(pid_controller_node ${catkin_LIBRARIES})
target_link_libraries(inverse_kinematics_node ${catkin_LIBRARIES})

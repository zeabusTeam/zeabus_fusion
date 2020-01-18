cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_localize )

find_package( catkin REQUIRED COMPONENTS 
    tf
    roscpp
    nav_msgs
    sensor_msgs
    zeabus_utility
)

find_package( zeabus_library REQUIRED COMPONENTS )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

add_executable( localize 
    src/localize/localize.cpp # Main file
    src/localize/calculate_angular_velocity.cpp # File algorithm sequence calculate angular velocity
    src/localize/target_service.cpp # File ros part about service to configure target state
    src/localize/calculate_linear_velocity.cpp # File algorithm sequence calculate linear velocity
    src/localize/updated_target_state.cpp # File algorithm update target state
)
add_dependencies( localize ${catkin_EXPORTED_TARGETS} )
target_link_libraries( localize
    ${zeabus_library_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( quaternion
    src/know_quaternion.cpp
)
target_link_libraries( quaternion
    ${zeabus_library_LIBRARIES}
)

add_executable( robot
    src/know_robot.cpp
)
target_link_libraries( robot
    ${zeabus_library_LIBRARIES}
)

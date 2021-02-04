cmake_minimum_required(VERSION 3.0)
project(base_estimation)

set(CMAKE_CXX_STANDARD 14)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

find_package(cartesian_interface REQUIRED)
find_package(catkin REQUIRED COMPONENTS visualization_msgs std_msgs message_generation)

include_directories(include/)

add_message_files(
    FILES
    ContactStatus.msg
    ContactsStatus.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

# compile library
add_library(ikbe SHARED
    src/base_estimation.cpp
    src/vertex_force_optimizer.cpp)

target_link_libraries(ikbe
    PRIVATE
    ${cartesian_interface_LIBRARIES}
    ${catkin_LIBRARIES}
    )

install(
    TARGETS ikbe
    DESTINATION lib
    )

find_package(xbot2 QUIET)

# compile plugin
if(${xbot2_FOUND})

    add_xbot2_plugin(base_estimation src/base_estimation_plugin.cpp src/contact_viz.cpp)

    target_link_libraries(base_estimation
        PRIVATE
        ikbe
        xbot2::xbot2_ros_support
        )

    install(
        TARGETS base_estimation
        DESTINATION lib
        )

endif()

# compile cartesio rt
add_subdirectory(cartesio)

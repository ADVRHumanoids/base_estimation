cmake_minimum_required(VERSION 3.0)
project(base_estimation)

set(CMAKE_CXX_STANDARD 14)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

find_package(cartesian_interface REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    visualization_msgs
    std_msgs
    message_generation
    #cartesio_acceleration_support

    ocs2_msgs
    )


include_directories(include/
      ${catkin_INCLUDE_DIRS})

add_message_files(
    FILES
    ContactStatus.msg
    ContactsStatus.msg
    ContactsWrench.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

catkin_package()

# compile library
add_library(ikbe SHARED
    src/base_estimation.cpp
    src/vertex_force_optimizer.cpp
    src/contact_estimation.cpp)

target_link_libraries(ikbe
    PUBLIC
    ${cartesian_interface_LIBRARIES}
    ${catkin_LIBRARIES}
    )

install(
    TARGETS ikbe
    DESTINATION lib
    )

find_package(xbot2 QUIET)

# compile plugin
if(FALSE)#${xbot2_FOUND})

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

# compile node
add_executable(base_estimation_node src/base_estimation_node.cpp)

target_link_libraries(base_estimation_node
    PRIVATE
    ikbe
    xbot2::xbot2
    )

install(
    TARGETS base_estimation_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# compile cartesio rt
#add_subdirectory(cartesio)

# install files
install(
    DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

cmake_minimum_required(VERSION 3.0)
project(base_estimation VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_TARGET_NAME ikbe) # set library name

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

find_package(cartesian_interface REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    visualization_msgs
    std_msgs
    message_generation)

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

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${LIBRARY_TARGET_NAME}
)

# compile library
add_library(${LIBRARY_TARGET_NAME} SHARED
    src/base_estimation.cpp
    src/vertex_force_optimizer.cpp
    src/contact_estimation.cpp)

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
    ${cartesian_interface_LIBRARIES}
    ${catkin_LIBRARIES}
    )

install(TARGETS ${LIBRARY_TARGET_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

find_package(xbot2 QUIET)

# compile plugin
if(${xbot2_FOUND})

    add_xbot2_plugin(base_estimation src/base_estimation_plugin.cpp src/contact_viz.cpp)

    target_link_libraries(base_estimation
        PRIVATE
        ${LIBRARY_TARGET_NAME}
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
    ${LIBRARY_TARGET_NAME}
    xbot2::xbot2
    )

install(
    TARGETS base_estimation_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# compile cartesio rt
add_subdirectory(cartesio)

# install files
install(
    DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

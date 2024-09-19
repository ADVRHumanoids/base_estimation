cmake_minimum_required(VERSION 3.10)
project(base_estimation)

set(CMAKE_CXX_STANDARD 20)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")
option(COMPILE_CARTESIO_PLUGIN OFF "Compile Cartesio RT plugin")

find_package(cartesian_interface REQUIRED)

find_package(estimation_utils REQUIRED)

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(tf2_msgs REQUIRED)

find_package(tf2_eigen REQUIRED)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactStatus.msg"
  "msg/ContactsStatus.msg"
  DEPENDENCIES
  std_msgs
)


# compile library
add_library(ikbe SHARED
    src/base_estimation.cpp
    src/vertex_force_optimizer.cpp
    src/contact_estimation.cpp)


target_link_libraries(ikbe
    PUBLIC
    cartesian_interface::CartesianInterface
    estimation_utils::payload_estimation
    )

install(
    TARGETS ikbe
    DESTINATION lib
    )

find_package(xbot2 QUIET)

# compile plugin
if(${xbot2_FOUND})

    add_xbot2_plugin(base_estimation_plugin src/base_estimation_plugin.cpp src/contact_viz.cpp)

    target_link_libraries(base_estimation_plugin
        PRIVATE
        ikbe
        xbot2::xbot2_ros2_support
        ${PROJECT_NAME}__rosidl_typesupport_cpp
        tf2_eigen::tf2_eigen
        tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
        )

    install(
        TARGETS base_estimation_plugin
        DESTINATION lib
        )
        
# compile node
add_executable(base_estimation_node src/base_estimation_node.cpp)

target_link_libraries(base_estimation_node
    PRIVATE
    ikbe
    xbot2::xbot2
    rclcpp::rclcpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    tf2_eigen::tf2_eigen
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    )

install(
    TARGETS base_estimation_node
    DESTINATION lib/${PROJECT_NAME}
    )

endif()



# compile gazebo tf publisher
# add_executable(gazebo_tf_publisher src/gazebo_tf_publisher.cpp)

# target_link_libraries(gazebo_tf_publisher
#     PRIVATE
#     ${catkin_LIBRARIES}
#     )

# install(
#     TARGETS gazebo_tf_publisher
#     DESTINATION lib/${PROJECT_NAME}
#     )


# install files
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    )

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()

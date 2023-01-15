find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(cartesio_acceleration_support REQUIRED)
add_xbot2_plugin(cartesio_plugin
    cartesio_rt.cpp)

add_dependencies(cartesio_plugin ${PROJECT_NAME}_generate_messages)
target_include_directories(cartesio_plugin PRIVATE ${cartesian_interface_INCLUDE_DIRS} ${cartesio_acceleration_support_INCLUDE_DIRS})
target_link_libraries(cartesio_plugin PRIVATE ${cartesian_interface_LIBRARIES} ${cartesio_acceleration_support_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS cartesio_plugin
    DESTINATION lib
    )



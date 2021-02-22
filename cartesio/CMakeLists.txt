find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)

add_xbot2_plugin(cartesio_plugin
    cartesio_rt.cpp)

target_link_libraries(cartesio_plugin PRIVATE ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS cartesio_plugin
    DESTINATION lib
    )



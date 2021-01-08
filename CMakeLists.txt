cmake_minimum_required(VERSION 3.0)
project(base_estimation)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")


find_package(xbot2 REQUIRED)
find_package(cartesian_interface REQUIRED)

# client
add_xbot2_plugin(base_estimation src/base_estimation.cpp)

target_link_libraries(base_estimation PRIVATE ${cartesian_interface_LIBRARIES} xbot2::xbot2_ros_support)

install(
    TARGETS base_estimation
    DESTINATION lib
    )



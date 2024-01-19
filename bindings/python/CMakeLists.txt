cmake_minimum_required(VERSION 3.15)

find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings (${pybind11_LIBRARIES})")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(pybase_estimation pyBaseEstimation.cpp)
    target_link_libraries(pybase_estimation PRIVATE ikbe)
    install(TARGETS pybase_estimation
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

else()
    message(STATUS "-----------------------------------pybind not found")
endif()


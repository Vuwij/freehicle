cmake_minimum_required(VERSION 3.0.2)
project(freehicle_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)


xacro_add_files(
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/freehicle/freehicle.xacro OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/urdf/freehicle/freehicle.urdf TARGET
    media_files
)

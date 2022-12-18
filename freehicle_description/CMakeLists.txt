cmake_minimum_required(VERSION 3.0.2)
project(freehiche_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)



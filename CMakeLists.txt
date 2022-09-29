project(px4_nmpc)
cmake_minimum_required(VERSION 3.10)

find_package(ACADO REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${ACADO_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

link_directories(${ACADO_LIBRARY_DIRS})

add_executable(px4_nmpc
    src/main.cpp
)
target_link_libraries(px4_nmpc ${ACADO_SHARED_LIBRARIES})

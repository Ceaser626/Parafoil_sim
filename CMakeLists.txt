cmake_minimum_required(VERSION 3.30)
project(parafoil_sim)

#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "D:\\Programming Pack\\vcpkg\\installed\\x64-mingw-dynamic")
find_package(Eigen3 REQUIRED)

set(PARAFOIL_INCLUDE_DIRECTORIES
    include
)
set(PARAFOIL_SOURCES
    src/sim.cpp
    src/simulation.cpp
    src/six_dof_parafoil.cpp
    src/utility.cpp
)
add_library(parafoil SHARED ${PARAFOIL_SOURCES})
target_include_directories(parafoil PUBLIC ${PARAFOIL_INCLUDE_DIRECTORIES})
target_link_libraries(parafoil Eigen3::Eigen)

add_executable(sim src/sim.cpp)
target_link_libraries(sim parafoil)

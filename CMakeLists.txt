cmake_minimum_required(VERSION 3.10)

project(ICAR_BECKHOFF_CPP)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(BOOST_INCLUDEDIR "/usr/include")

include_directories(/root/SOEM/install/include)

add_executable(beckhoff beckhoff.cpp)

target_link_libraries(beckhoff /root/SOEM/install/lib/libsoem.a )
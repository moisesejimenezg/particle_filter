project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

file(GLOB HEADERS src/*.h)
file(GLOB HEADERS_HPP src/*.hpp)

set(sources src/particle_filter.cpp ${HEADERS} ${HEADERS_HPP})
set(main src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter ${main} ${sources})

target_link_libraries(particle_filter z ssl uv uWS)

add_executable(test ${sources} test/test_particle_filter.cpp)
target_link_libraries(test z ssl uv uWS)

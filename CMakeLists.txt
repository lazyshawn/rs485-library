cmake_minimum_required(VERSION 2.8.9)
project(rs485-library)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# master
add_executable(main
  src/main.cpp
  src/rs485_driver.cpp
)
target_include_directories(main PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)


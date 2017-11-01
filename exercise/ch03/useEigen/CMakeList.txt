cmake_minimum_required(VERSION 2.8)
project(useEigenTest)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O1")

include_directories("/usr/include/eigen3")

add_executable(eigenMatrix eigenMatrix.cpp)
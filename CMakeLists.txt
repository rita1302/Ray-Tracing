cmake_minimum_required(VERSION 3.5)
project(rt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

add_executable(rt mashgraph.cpp pch.h pixel.h)

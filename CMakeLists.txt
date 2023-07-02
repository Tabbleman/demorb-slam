cmake_minimum_required(VERSION 3.15)

project(demoorb)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(
  DEMO SHARED 
  ${PROJECT_SOURCE_DIR}/frame.cc 
)


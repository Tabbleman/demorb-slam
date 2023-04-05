cmake_minimum_required(VERSION 3.10)
project(demoslam)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(EXECUTABLE_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)

#find dependencies
find_package(Eigen3 3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Boost 1.68 REQUIRED)
find_package(g2o REQUIRED)
find_package(DBoW3 REQUIRED)

#include dependencies libraries
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${DBoW_INCLUDE_DIRS}
)

set(THIRD_LIBS
${OpenCV_LIBS}
${Eigen3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBS}
g2o_core g2o_stuff 
${DBoW_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED
  src/System.cc
)
target_link_libraries(${PROJECT_NAME} 
  ${THIRD_LIBS}
)

add_executable(start_slam app/start.cc)
target_link_libraries(start_slam ${PROJECT_NAME})

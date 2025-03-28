cmake_minimum_required(VERSION 3.13) 
project(simple_example)             
set(CMAKE_CXX_STANDARD 14)           
find_package(xtensor REQUIRED)
include_directories(include 
                    include/utils)
# include_directories(include
#                     include/utils)
# set(SOURCE_FILES src/main.cpp)
# add_executable(simple_example ${SOURCE_FILES})
add_executable(test_xt src/models/test_xtensor.cpp)
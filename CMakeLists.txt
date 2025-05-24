cmake_minimum_required(VERSION 3.10)

# Set project name and version
project(NanoCDR VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(BUILD_TESTS ON)

if(BUILD_TESTS)
    enable_testing()
    add_executable(test_nanocdr test/test_nanocdr.cpp)
    target_include_directories(test_nanocdr PRIVATE include)
    add_test(NAME test_nanocdr COMMAND test_nanocdr)
endif()



cmake_minimum_required(VERSION 3.10)

# Set project name and version
project(nanocdr VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# header only target
add_library(nanocdr INTERFACE)
target_include_directories(nanocdr INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS nanocdr
    EXPORT nanocdr
)

###########################################################

# Set compiler flags

option(BUILD_TESTS "Build tests" ON)

if(BUILD_TESTS)
    enable_testing()
    add_executable(test_nanocdr test/test_nanocdr.cpp)
    target_include_directories(test_nanocdr PRIVATE include)
    add_test(NAME test_nanocdr COMMAND test_nanocdr)
endif()

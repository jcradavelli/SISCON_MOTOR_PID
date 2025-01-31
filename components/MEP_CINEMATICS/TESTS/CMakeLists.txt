cmake_minimum_required(VERSION 3.10)
project(Teste_MEP_CINEMATICS)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_SYSTEM_NAME "Windows")
set(CMAKE_SYSTEM_PROCESSOR "x86_64")

add_definitions(-DLOG_BY_WINDOWS=true)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(Teste_MEP_CINEMATICS ../MEP_CINEMATICS.c tests.c)
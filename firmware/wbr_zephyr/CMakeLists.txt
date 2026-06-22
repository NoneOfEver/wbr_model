cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wbr_zephyr LANGUAGES C CXX)

set(WBR_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
include(${WBR_ROOT}/control_core/sources.cmake)

target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cc
    ${CMAKE_CURRENT_LIST_DIR}/src/control_thread.cc
    ${CMAKE_CURRENT_LIST_DIR}/port/hardware_stub.cc
    ${WBR_CONTROL_CORE_SOURCES})

target_include_directories(app PRIVATE
    ${WBR_ROOT}/control_core/include
    ${CMAKE_CURRENT_LIST_DIR}/include)

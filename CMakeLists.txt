cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T LANGUAGES C CXX OBJCXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/simulate)

set(MUJOCO_SRC "/Users/panpoming/Documents/mujoco_ws/mujoco" CACHE PATH "MuJoCo source path")
set(MUJOCO_BUILD "/Users/panpoming/Documents/mujoco_ws/mujoco/build" CACHE PATH "MuJoCo build path")
set(MUJOCO_LIB "${MUJOCO_BUILD}/lib/libmujoco.dylib")

include_directories(${MUJOCO_SRC}/include)
find_package(glfw3 REQUIRED)

add_executable(basic basic.cc)
target_link_libraries(basic PRIVATE ${MUJOCO_LIB} glfw)
target_link_options(basic PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_executable(ik_viewer ik_viewer.cc)
target_link_libraries(ik_viewer PRIVATE ${MUJOCO_LIB} glfw)
target_link_options(ik_viewer PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

file(GLOB SIMULATE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.mm)
add_executable(simulate ${SIMULATE_SRC})
target_link_libraries(simulate PRIVATE ${MUJOCO_LIB} glfw ${MUJOCO_BUILD}/lib/liblodepng.a)
target_link_options(simulate PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

if (APPLE)
    target_link_libraries(basic PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL")
    target_link_libraries(ik_viewer PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL")
    target_link_libraries(simulate PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL" "-framework CoreVideo")
endif()

cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T LANGUAGES C CXX OBJCXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

add_executable(lqr_open_loop_test lqr_open_loop_test.cc)
target_link_libraries(lqr_open_loop_test PRIVATE ${MUJOCO_LIB})
target_link_options(lqr_open_loop_test PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_executable(lqr_identification lqr_identification.cc)
target_link_libraries(lqr_identification PRIVATE ${MUJOCO_LIB})
target_link_options(lqr_identification PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_executable(kinematics_checker kinematics_checker.cc)
target_link_libraries(kinematics_checker PRIVATE ${MUJOCO_LIB})
target_link_options(kinematics_checker PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_executable(reduced_model_check reduced_model_check.cc)
target_link_libraries(reduced_model_check PRIVATE ${MUJOCO_LIB})
target_link_options(reduced_model_check PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_executable(reduced_identification reduced_identification.cc)
target_link_libraries(reduced_identification PRIVATE ${MUJOCO_LIB})
target_link_options(reduced_identification PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

add_subdirectory(control_core)

add_executable(control_core_smoke_test
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/host/control_core_smoke_test.cc)
target_link_libraries(control_core_smoke_test PRIVATE wbr_control_core)

add_library(wbr_mujoco_platform STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/src/actuator_io.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/src/leg_io.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/src/observation_io.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/src/state_estimator.cc)
target_include_directories(wbr_mujoco_platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/include)
target_link_libraries(wbr_mujoco_platform PUBLIC wbr_control_core)

set(WBR_CONTROLLER_V2_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate/wbr_controller_v2.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/mujoco/src/controller_binding.cc)

add_executable(wbr_v2_closed_loop_test
    wbr_v2_closed_loop_test.cc
    ${WBR_CONTROLLER_V2_SOURCES})
target_link_libraries(wbr_v2_closed_loop_test PRIVATE
    wbr_mujoco_platform ${MUJOCO_LIB})
target_link_options(wbr_v2_closed_loop_test PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

file(GLOB SIMULATE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate/*.mm)
list(APPEND SIMULATE_SRC ${WBR_CONTROLLER_V2_SOURCES})
list(REMOVE_DUPLICATES SIMULATE_SRC)
add_executable(simulate ${SIMULATE_SRC})
target_link_libraries(simulate PRIVATE
    wbr_mujoco_platform
    ${MUJOCO_LIB} glfw ${MUJOCO_BUILD}/lib/liblodepng.a)
target_link_options(simulate PRIVATE "-Wl,-rpath,${MUJOCO_BUILD}/lib")

if (APPLE)
    target_link_libraries(basic PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL")
    target_link_libraries(ik_viewer PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL")
    target_link_libraries(simulate PRIVATE "-framework Cocoa" "-framework IOKit" "-framework OpenGL" "-framework CoreVideo")
endif()

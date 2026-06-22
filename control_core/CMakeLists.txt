include(${CMAKE_CURRENT_LIST_DIR}/sources.cmake)

add_library(wbr_control_core STATIC ${WBR_CONTROL_CORE_SOURCES})
target_include_directories(wbr_control_core PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include)


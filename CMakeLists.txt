cmake_minimum_required(VERSION 3.22)

set(RUKA3_BOARD "g431" CACHE STRING "Target board: g431 or g474")
set_property(CACHE RUKA3_BOARD PROPERTY STRINGS g431 g474)
set(RUKA3_JOINT_INDEX "6" CACHE STRING "Robot joint index (1..6)")

if(RUKA3_BOARD STREQUAL "g431")
    if(NOT RUKA3_JOINT_INDEX MATCHES "^[56]$")
        message(FATAL_ERROR "g431 supports joints 5..6")
    endif()
elseif(RUKA3_BOARD STREQUAL "g474")
    if(NOT RUKA3_JOINT_INDEX MATCHES "^[1-4]$")
        message(FATAL_ERROR "g474 supports joints 1..4")
    endif()
else()
    message(FATAL_ERROR "RUKA3_BOARD must be g431 or g474")
endif()

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/Boards/${RUKA3_BOARD}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "RUKA3 ARM toolchain")
endif()

project(RUKA3 NONE)
add_subdirectory("Boards/${RUKA3_BOARD}")

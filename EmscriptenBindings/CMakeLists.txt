cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)


set(EMSCRIPTEN_ROOT $ENV{EMSDK}/upstream/emscripten CACHE STRING "Emscripten path")
set(CMAKE_TOOLCHAIN_FILE ${EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake)

set(ENTRY_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/recast-navigation.h)
set(RECAST_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

project("recast-navigation-wasm")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AS_JS "compile as js" OFF)
option(RELEASE "compile as js" OFF)

file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

file(GLOB_RECURSE RECASTDETOUR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Source/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Source/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Source/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Source/*.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCacheBuilder/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Include
)

if(AS_JS)
  SET(EXE_NAME "recast-navigation")
else()
  SET(EXE_NAME "recast-navigation-wasm")
endif()

#ADD_LIBRARY(${EXE_NAME} ${SRC_FILES} ${RECASTDETOUR_FILES})


# Build Release by default
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type")


set(EMCC_GLUE_ARGS
  -c
  -I${RECAST_SRC_DIR}
  -I${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Include
)


add_executable(${EXE_NAME} ${SRC_FILES} ${RECASTDETOUR_FILES})

set(CMAKE_FLAG "--bind  -s EXPORT_NAME=Recast -s ALLOW_MEMORY_GROWTH=1 -O3 -s EXPORTED_FUNCTIONS=_malloc,_free -s MODULARIZE=1 -s NO_DYNAMIC_EXECUTION=1 ")

string(APPEND CMAKE_FLAG " -s FILESYSTEM=0 -s NO_FILESYSTEM=1 -s SUPPORT_LONGJMP=0  ")

if(AS_JS)
  string(APPEND CMAKE_FLAG "-s WASM=0")
else()
  string(APPEND CMAKE_FLAG "-s WASM=1")
endif()

# if(CMAKE_BUILD_TYPE STREQUAL "debug")
  string(APPEND CMAKE_FLAG " -g3 ")
# endif()




set_target_properties(${EXE_NAME} PROPERTIES
	LINK_FLAGS "${CMAKE_FLAG}"
)
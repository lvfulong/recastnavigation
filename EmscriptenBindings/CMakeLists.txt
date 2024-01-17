cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

#FIND_PACKAGE(Python3)
#set(PYTHON ${Python3_EXECUTABLE} CACHE STRING "Python path")
set(EMSCRIPTEN_ROOT $ENV{EMSDK}/upstream/emscripten CACHE STRING "Emscripten path")
set(CMAKE_TOOLCHAIN_FILE ${EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake)
#set(WEBIDL_BINDER_SCRIPT ${EMSCRIPTEN_ROOT}/tools/webidl_binder.py)

#set(RECAST_FRONT_MATTER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/front-matter.js)
#set(RECAST_IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/recast-navigation.idl)
set(ENTRY_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/recast-navigation.h)
set(RECAST_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

project("recast-navigation-wasm")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

SET(EXE_NAME "recast-navigation")

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
  #-include${ENTRY_HEADER_FILE}
)


#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EMCC_WASM_ESM_ARGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EMCC_WASM_ESM_ARGS}")

add_executable(${EXE_NAME} ${SRC_FILES} ${RECASTDETOUR_FILES})
#set_target_properties(${EXE_NAME}  PROPERTIES CXX_STANDARD 17)
#target_compile_options(${EXE_NAME}  PUBLIC "$<$<BOOL:${MSVC}>:/permissive->")
#target_link_options(${EXE_NAME}  PUBLIC "--bind -s EXPORT_NAME=Recast -s MODULARIZE=1")
set_target_properties(${EXE_NAME} PROPERTIES
	LINK_FLAGS "--bind  -s EXPORT_NAME=Recast -s ALLOW_MEMORY_GROWTH=1 -s EXPORTED_FUNCTIONS=_malloc,_free -s MODULARIZE=1 -g"
)
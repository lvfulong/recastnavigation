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
    ${CMAKE_SOURCE_DIR}/*.h
    ${CMAKE_SOURCE_DIR}/*.cpp
)

file(GLOB_RECURSE RECASTDETOUR_FILES
    ${CMAKE_SOURCE_DIR}/../Detour/*
    ${CMAKE_SOURCE_DIR}/../DetourCrowd/*
    ${CMAKE_SOURCE_DIR}/../DetourTileCache/*
    ${CMAKE_SOURCE_DIR}/../Recast/*
    #${CMAKE_SOURCE_DIR}/../RecastDemo/Source/ChunkyTriMesh.cpp
    #${CMAKE_SOURCE_DIR}/../RecastDemo/Contrib/fastlz/fastlz.c
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCacheBuilder/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../RecastDemo/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/../DebugUtils/Include
)

SET(EXE_NAME "recast-navigation")

ADD_LIBRARY(${EXE_NAME} ${SRC_FILES} ${RECASTDETOUR_FILES})

# Build Release by default
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type")

set(EMCC_ARGS
  #-fno-rtti
  -flto
  #--extern-pre-js ${RECAST_FRONT_MATTER_FILE}
  #--post-js glue.js
  -O3
  -s INITIAL_MEMORY=67108864 # 64*1024*1024 = 64MB
  -s ALLOW_MEMORY_GROWTH=1
  -s ALLOW_TABLE_GROWTH=1
  -s EXPORTED_FUNCTIONS=["_malloc","_free"]
  -s EXPORTED_RUNTIME_METHODS=["UTF8ToString","addFunction"]
  -s EXPORT_NAME="Recast"
  -s MODULARIZE=1
  -s ENVIRONMENT='web'
  -s NO_EXIT_RUNTIME=1
  -s NO_FILESYSTEM=1
  -s FILESYSTEM=0
  -s EXPORT_ES6=1
  -s BINARYEN_IGNORE_IMPLICIT_TRAPS=1 
  # Start: Uncomment for debugging
  # -O0
  # -g
  # -s ASSERTIONS=2
  # End: Uncomment for debugging
)

if(${CLOSURE})
  LIST(APPEND EMCC_ARGS
    --closure 1
    -s IGNORE_CLOSURE_COMPILER_ERRORS=1)
else()
  LIST(APPEND EMCC_ARGS
    -s NO_DYNAMIC_EXECUTION=1)
endif()

set(EMCC_WASM_ESM_ARGS ${EMCC_ARGS}
  -s WASM=1)

set(EMCC_WASM_COMPAT_ESM_ARGS ${EMCC_ARGS}
  -s SINGLE_FILE=1
  -s WASM=1)

set(EMCC_GLUE_ARGS
  -c
  -I${RECAST_SRC_DIR}
  -I${CMAKE_CURRENT_SOURCE_DIR}/../Detour/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../DetourCrowd/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../DetourTileCache/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../Recast/Include
  -I${CMAKE_CURRENT_SOURCE_DIR}/../DebugUtils/Include
  #-include${ENTRY_HEADER_FILE}
)

# ES6 WASM
add_custom_command(
  OUTPUT ${EXE_NAME}.wasm.js ${EXE_NAME}.wasm.wasm
  COMMAND emcc --bind -Wl,--no-whole-archive lib${EXE_NAME}.a ${EMCC_WASM_ESM_ARGS} -o ${EXE_NAME}.wasm.js -Wl,--no-whole-archive
  DEPENDS  ${EXE_NAME}
  COMMENT "Building ${EXE_NAME} webassembly"
  VERBATIM)
add_custom_target(${EXE_NAME}-wasm ALL DEPENDS ${EXE_NAME}.wasm.js ${EXE_NAME}.wasm.wasm)


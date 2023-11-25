#!/bin/sh
mkdir -p ./build
mkdir -p dist
cmake ./EmscriptenBindings  -B ./build/release -DTARGET_BUILD_PLATFORM=emscripten -DCMAKE_TOOLCHAIN_FILE=~/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=./dist  -DCMAKE_BUILD_TYPE=release -DCMAKE_CROSSCOMPILING_EMULATOR=/home/ubuntu/emsdk/node/16.20.0_64bit/bin/node
cd ./build/release
emmake make -j4

#!/bin/bash
rm -rf wasm_build
cmake .  -B ./build/debug -DTARGET_BUILD_PLATFORM=emscripten -DCMAKE_TOOLCHAIN_FILE=~/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=./install/emscripten/recastnavigation -DRECASTNAVIGATION_EXAMPLES=OFF -DRECASTNAVIGATION_TESTS=OFF -DRECASTNAVIGATION_DEMO=OFF -DCMAKE_BUILD_TYPE=debug -DCMAKE_CROSSCOMPILING_EMULATOR=/home/ubuntu/emsdk/node/16.20.0_64bit/bin/node
cd ./build/debug
emmake make -j4
cd ../../
mkdir wasm_build
cd wasm_build
cp ../install/emscripten/recastnavigation/bin/recastnavigation.debug.js .

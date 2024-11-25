#!/bin/sh
rm -rf build
mkdir -p ./build
mkdir -p dist
cmake ./EmscriptenBindings  -B ./build/debug  -G "MinGW Makefiles" -DTARGET_BUILD_PLATFORM=emscripten -DCMAKE_TOOLCHAIN_FILE=~/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=./dist  -DCMAKE_BUILD_TYPE=debug -DCMAKE_CROSSCOMPILING_EMULATOR=/home/ubuntu/emsdk/node/16.20.0_64bit/bin/node
cd ./build/debug
emmake make -j4
cd ../../dist
cp ../build/debug/recast-navigation-wasm.js .
cp ../build/debug/recast-navigation-wasm.wasm .
cd ../


# cmake ./EmscriptenBindings  -B ./build/debug  -G "MinGW Makefiles" -DAS_JS=1 -DTARGET_BUILD_PLATFORM=emscripten -DCMAKE_TOOLCHAIN_FILE=~/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=./dist  -DCMAKE_BUILD_TYPE=debug -DCMAKE_CROSSCOMPILING_EMULATOR=/home/ubuntu/emsdk/node/16.20.0_64bit/bin/node
# cd ./build/debug
# emmake make -j4
# cd ../../dist
# cp ../build/debug/recast-navigation.js .
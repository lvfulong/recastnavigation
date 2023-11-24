#!/bin/sh

mkdir -p ./build
mkdir -p dist


emcmake cmake -B build -S EmscriptenBindings -DCMAKE_BUILD_TYPE=Debug
cmake --build build



cp ./build/recast-navigation.* ./dist/
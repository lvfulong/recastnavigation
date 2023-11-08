
#include <emscripten.h>
#include <emscripten/bind.h>


using namespace emscripten;


EMSCRIPTEN_BINDINGS(recastnavigation) {
    constant("TEST", 9999);
}


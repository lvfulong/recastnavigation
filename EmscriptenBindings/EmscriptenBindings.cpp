
#include <emscripten.h>
#include <emscripten/bind.h>


using namespace emscripten;

float lerp(float a, float b, float t) {
    return (1 - t) * a + t * b;
}
EMSCRIPTEN_BINDINGS(recastnavigation) {
    constant("TEST", 9999);
	function("lerp", &lerp);
}


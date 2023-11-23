#file(GLOB SOURCES ./*.cpp ./*.h)


SET(EMSCRIPTEN_USE_ASSERTIONS "1")
SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=RECASTNAVIGATION -s ALLOW_MEMORY_GROWTH=1")


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)


set(SOURCES)

list(APPEND SOURCES
	EmscriptenBindings.cpp
)
	
#ADD_EXECUTABLE(EmscriptenBindings ${SOURCES})
ADD_EXECUTABLE(EmscriptenBindings ${SOURCES})

#file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
SET_TARGET_PROPERTIES(EmscriptenBindings PROPERTIES
	LINK_FLAGS "${EMSCRIPTEN_BASE_OPTIONS} -s ASSERTIONS=${EMSCRIPTEN_USE_ASSERTIONS}"
)


add_dependencies(EmscriptenBindings DebugUtils Detour DetourCrowd DetourTileCache Recast)


install(TARGETS EmscriptenBindings
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
#install(DIRECTORY Bin/Meshes DESTINATION bin)
#install(DIRECTORY Bin/TestCases DESTINATION bin)
#install(FILES Bin/DroidSans.ttf DESTINATION bin)

set_target_properties(EmscriptenBindings PROPERTIES OUTPUT_NAME "recastnavigation.${CMAKE_BUILD_TYPE}")

SET(DISTRO_LIBS EmscriptenBindings)
INSTALL(
	TARGETS ${DISTRO_LIBS}
	EXPORT EmscriptenBindings
	DESTINATION $<$<CONFIG:debug>:${CMAKE_INSTALL_LIBDIR}/debug>$<$<CONFIG:release>:${CMAKE_INSTALL_LIBDIR}/release>$<$<CONFIG:checked>:${CMAKE_INSTALL_LIBDIR}/checked>$<$<CONFIG:profile>:${CMAKE_INSTALL_LIBDIR}/profile>	
	)



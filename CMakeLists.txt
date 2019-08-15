cmake_minimum_required(VERSION 3.1...3.14)

project(geomorph
    VERSION 1.0
    DESCRIPTION "library to create realistic large-scale landscapes"
    LANGUAGES CXX)

set(HEADERS
    "include/geomorph/erosion.h"
    "include/geomorph/map.h"
    "include/geomorph/map.inl.h"
    "include/geomorph/map_fwd.h"
    "include/geomorph/math_utils.h"
    "include/geomorph/noise.h"
    "include/geomorph/vec2f.h"
    "include/geomorph/water.h")

set(SOURCES
    "src/erosion.cpp"    
    "src/math_utils.cpp"
    "src/noise.cpp"
    "src/vec2f.cpp"
    "src/water.cpp")

add_library(geomorph-lib STATIC ${SOURCES} ${HEADERS})
target_include_directories(geomorph-lib PUBLIC "include")
set_target_properties(geomorph-lib PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)

add_executable(geomorph-test "test/main.cpp")
target_link_libraries(geomorph-test PRIVATE geomorph-lib)

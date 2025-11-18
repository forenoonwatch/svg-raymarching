# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

add_subdirectory(Catch2)
add_subdirectory(glfw)
add_subdirectory(glm)
add_subdirectory(simdjson)

# glad
add_library(glad STATIC "")
target_sources(glad PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/glad/src/glad.c")
target_include_directories(glad PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/glad/include")

# nanovg
add_library(nanovg STATIC "")
target_sources(nanovg PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/nanovg/src/nanovg.c")
target_include_directories(nanovg PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/nanovg/src")

# nanosvg
add_library(nanosvg INTERFACE "")
target_include_directories(nanosvg INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/nanosvg/src")

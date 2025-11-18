cmake_minimum_required(VERSION 3.14)

project(
	raymarch VERSION 1.0
	LANGUAGES CXX C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

add_subdirectory(third_party)

add_executable(raymarch "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
set_target_properties(raymarch PROPERTIES
	CXX_STANDARD 23
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)
target_link_libraries(raymarch PRIVATE glfw glad glm::glm nanovg nanosvg)

target_sources(raymarch PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/martinez.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/shader.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/svg.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/svg_subdivision.cpp"
)

add_executable(testcases "")
set_target_properties(testcases PROPERTIES
	CXX_STANDARD 23
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)
target_link_libraries(testcases PRIVATE glm::glm Catch2::Catch2WithMain simdjson)

target_sources(testcases PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/martinez.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/test_martinez.cpp"
)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/martinez/test/featureTypes"
	DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/martinez/test/fixtures"
	DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/martinez/test/genericTestCases"
	DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

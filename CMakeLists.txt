cmake_minimum_required(VERSION 3.16)
project(TotalCurvature)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(core)
igl_include(glfw)
igl_include(predicates)
igl_include(restricted triangle)
igl_include(copyleft cgal)

# polyscope
add_subdirectory("polyscope")

# openmp
find_package(OpenMP REQUIRED)

# Add your project files
file(GLOB SRC_FILES main.cpp)
add_executable(TotalCurvature ${SRC_FILES})
target_link_libraries(TotalCurvature PUBLIC igl::glfw igl_restricted::triangle igl_copyleft::cgal polyscope PRIVATE OpenMP::OpenMP_CXX)

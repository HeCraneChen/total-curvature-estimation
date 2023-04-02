# discrete total curvature

A universal total curvature estimation method that works for both triangle meshes and point clouds.

## Dependencies

- STL
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix data structure
- [openmp](http://polyscope.run/) for parallelization
- [libigl](http://libigl.github.io/libigl/) for 3D visualizations and rendering
- [Polyscope](http://polyscope.run/) for 3D visualizations and rendering

We referred to the [libigl-example-project](https://github.com/HeCraneChen/libigl-example-project) to download libigl and its dependencies:
The CMake build system will automatically download libigl and its dependencies using
[CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html),
thus requiring no setup on your part.

To use a local copy of libigl rather than downloading the repository via FetchContent, you can use
the CMake cache variable `FETCHCONTENT_SOURCE_DIR_LIBIGL` when configuring your CMake project for
the first time:
```
cmake -DFETCHCONTENT_SOURCE_DIR_LIBIGL=<path-to-libigl> ..
```
When changing this value, do not forget to clear your `CMakeCache.txt`, or to update the cache variable
via `cmake-gui` or `ccmake`.

## OS

The code is developed on MacOS 12.6.

## Compile

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake ..
    make

This should find and build the dependencies and create a `example_bin` binary.

## Run

From within the `build` directory just issue:

    ./TotalCurvature

A glfw app should launch displaying the point cloud of a cow, rendered with color representing total curvature of each point.

# total curvature estimation

A universal total curvature estimation method that works for both triangle meshes and point clouds. For details, see the 2023 paper by Crane Chen under the supervision of Misha Kazhdan.
![mesh_pcd_curvature](https://user-images.githubusercontent.com/33951209/229395487-efa580f7-9e28-498d-9265-af09d75f6d5c.png)

## Comparison with other popular libraries
![teaser_bright](https://user-images.githubusercontent.com/33951209/229387054-371fa8e9-1ef2-4552-81e3-af6927ee99dc.png)

## Dependencies

- [STL](https://www.geeksforgeeks.org/the-c-standard-template-library-stl/)
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix data structures
- [openmp](http://polyscope.run/) for parallelization
- [libigl](http://libigl.github.io/libigl/) for mesh data structures and geometry processing tools
- [polyscope](http://polyscope.run/) for 3D visualizations and rendering

## OS

The code was developed on MacOS 12.6.

## Compile

Fetch the code with dependencies:

    git clone https://github.com/HeCraneChen/total-curvature-estimation.git --recursive

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake -DFETCHCONTENT_SOURCE_DIR_LIBIGL=../libigl ..
    make

This should find and build the dependencies and create a `example_bin` binary.

## Run

From within the `build` directory just issue:

    ./TotalCurvature --in ../example_data/cow.ply --out ../results/cow_mesh.txt --format mesh
    ./TotalCurvature --in ../example_data/cow.ply --out ../results/cow_cloud.txt --format point_cloud

A glfw app should launch displaying a cow, rendered with color representing total curvature of each point. Results of the calculated curvature will be saved in a txt file. Note that the values in the visualizer has been rescaled for better visual effect. Refer to the output txt file for the calculated total curvature values.

## Citation

(to be updated)

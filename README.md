# discrete total curvature estimation

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
We verified the compilation and running of the code on Ubuntu 18.04.
The compilation and running process on MacOS and Ubuntu are stated as below.
We also verifie the compilation and running of the code on Windows with Visual Studio 2022, where we accessed CMake through Visual Studio. This procedure is slightly different from other OS. We demostrated it separately.

## Compile

Fetch the code with dependencies:

    git clone https://github.com/HeCraneChen/total-curvature-estimation.git --recursive

Compile this project using the standard cmake routine:

    cd total-curvature-estimation
    mkdir build
    cd build
    cmake ..
    make

The above commands use FetchContent of CMake to automatically download libigl and its dependencies. If that did not work for you, try to point to the local libigl. In that case, use

    cmake -DFETCHCONTENT_SOURCE_DIR_LIBIGL=../libigl ..
to replace

    cmake ..

## Run

From within the `build` directory, for triangle mesh, just issue:

    ./TotalCurvature --in ../example_data/cow.ply --out ../results/cow_mesh.txt --format mesh
    
For point cloud, just issue:

    ./TotalCurvature --in ../example_data/cow_points.ply ../example_data/cow_normals.ply --out ../results/cow_cloud.txt --format point_cloud

A glfw app should launch displaying a cow, rendered with color representing total curvature of each point. Results of the calculated curvature will be saved in a txt file. Note that the values in the visualizer has been rescaled for better visual effect. Refer to the output txt file for the calculated total curvature values.

## Visual Studio Users on Windows


## Citation

(coming soon)

# discrete total curvature estimation

A universal total curvature estimation method that works for both triangle meshes and point clouds. For details, see the [SIGGRAPH 2023 paper](https://arxiv.org/abs/2305.12653) by Crane Chen under the supervision of Misha Kazhdan.

This codebase is in libigl-style. We also implemented another version for open3d users, the open3d-style codebase of our paper can be found [here](https://github.com/HeCraneChen/open3d-discrete-total-curvature.git).

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
We also verifie the compilation and running of the code on Windows 10 with Visual Studio 2022, where we accessed CMake through Visual Studio. 
## MacOS and Ubuntu

**Compile**

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

If you are on a M1/M2/M3 chip Macbook, you are likely to run into issues with the openmp library. This is because the new Macbook put libomp in /opt/homebrew/, but CMake might not look in there. CMake looks at /usr/local/lib/ which does not exist. The fix is run the following:

    sudo mkdir -p /usr/local/lib /usr/local/include
    sudo ln -s $(brew --prefix libomp)/lib/libomp.dylib /usr/local/lib/libomp.dylib
    sudo ln -s $(brew --prefix libomp)/include/* /usr/local/include/
    cmake ..

**Run**

From within the `build` directory, for triangle mesh, just issue:

    ./TotalCurvature --in ../example_data/cow.ply --out ../results/cow_mesh.txt --format mesh
    
For point cloud, just issue:

    ./TotalCurvature --in ../example_data/cow_points.ply ../example_data/cow_normals.ply --out ../results/cow_cloud.txt --format point_cloud

A glfw app should launch displaying a cow, rendered with color representing total curvature of each point. Results of the calculated curvature will be saved in a txt file. Note that the values in the visualizer has been rescaled for better visual effect. Refer to the output txt file for the calculated total curvature values.

## Windows with Visual Studio

**Compile**

Open the Visual Studio IDE, and click the following

`Open a local folder` and open the total-curvature-estimation folder cloned from this repo

`File`  `Open`  `CMake...` and open the CMakeLists.txt

`Build`  `Build All`

**Run**

    cd total-curvature-estimation
    
    mkdir build
    
    scp ./out/build/x64-Debug/TotalCurvature.exe ./build/TotalCurvature.exe
    
    scp ./out/build/x64-Debug/_deps/gmp-src/lib/libgmp-10.dll ./build/libgmp-10.dll
    
    cd build
    
    TotalCurvature --in ../example_data/cow.ply --out ../results/cow_mesh.txt --format mesh
    
    TotalCurvature --in ../example_data/cow_points.ply ../example_data/cow_normals.ply --out ../results/cow_cloud.txt --format point_cloud
    

## Citation

```bibtex
@inproceedings{10.1145/3587421.3595439,
author = {Chen, Crane He},
title = {Estimating Discrete Total Curvature with Per Triangle Normal Variation},
year = {2023},
isbn = {9798400701436},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3587421.3595439},
doi = {10.1145/3587421.3595439},
booktitle = {ACM SIGGRAPH 2023 Talks},
articleno = {56},
numpages = {2},
location = {Los Angeles, CA, USA},
series = {SIGGRAPH '23}
}
```

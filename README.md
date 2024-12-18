# LC-CurveModel: Joint camera-lidar spatial curve modeling
<!-- markdownlint-disable MD047 -->
The purpose of this repository is to reconstruct complete spatial models of power lines using LiDAR and cameras. We treat power lines as spatial curves and reconstruct them using both parabolic equations and catenary equations. Using simulation results as an example, the reconstruction results can be seen in the following two images.

![9-degree](/image/9-degree.jpg)

![curve-fitting](/image/curve-fitting.png)

## Structure  

lc_core       -- core algorithm module  
lc_preprocess -- pre-process module  
lc_simulation -- simple simulation module  
lc_tools      -- auxiliary python scripts

## Dependencies

- Eigen(3.4.0 or other version)
- OpenCV(4.2.0 or other version)
- Ceres(2.1.0 or other version)
- GLOG/GFLAGS
- yaml-cpp

## Installation

1. Install the required dependencies.

2. Clone the repository:

    ```bash
    git clone https://github.com/zzzzyp-sgg/LC-CurveModel.git
    ```

3. Build the program:

    ```bash
    mkdir build && cd build
    cmake ..
    make -j8
    ```

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

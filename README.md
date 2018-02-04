# Align Trajectory

This code calculates the alignment between two trajectories with x,y,z ccordinates, developed and tested under Ubuntu 14.04. A transformation matrix (rotation, scale, and trasnalation) and the alignment error is calculated.

To know more about the alignment error, also known as the Absoulte Trajectory Error (ATE), see [1]. To know more about the transformation matrix, see [2].

## Make and Run
To compile:

* cd project_dir
* mkdir build
* cd build
* cmake ..
* make

To run:

* cd ../bin
* ./align ../examples/cam0_gt.visim  ../examples/orbslam_traj.txt 

### Prerequisites
* CMake 2.8
* Eigen3
* Boost (sudo apt-get install libboost-all-dev)
* Pangolin

## Authors

* **Sajad Saeedi** 

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## License
[1] J. Sturm and N. Engelhard and F. Endres and W. Burgard and D. Cremers}, "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012.

[2] B. Horn, "Closed-form solution of absolute orientation using unit quaternions", JOSA 1987

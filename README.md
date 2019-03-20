# Lidar Calibration

Lidar Calibration is one of the most important steps before using any kind of lidars for any kind of use.
Here we try to implement 2 research papers for unsupervised multi beam lidar  calibration for autonomous vehicles. The refernces are below.

This repository is a code for georeferencing a multi beam lidar

## Usage

I am too lazy to give the entire package and I believe others should try to make it
The steps are a bit complicated
```
Make a ros package ; 
// it sholud contain the proper dependencies which include pcl, Eigen
paste it in the src and build it with proper launch file.
just run it on an approx 15s bag file
```
The lidar_planarity.cpp code requires a pointcloud to have x,y,z,ring
>reference 1

The lidar_calibrator.cpp code requires a pointcloud to have x,y,z,r,theta,phi,ring
>reference 2
## Referneces
1) Unsupervised Calibration for Multi-beam Lasers

>Jesse Levinson, Sebastian Thrun

2) Point cloud refinement with self-calibration of a mobile
multibeam lidar sensor

>Houssem Nouira, Jean-Emmanuel Deschaud, François Goulette


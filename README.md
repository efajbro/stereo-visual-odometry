# Stereo Visual Odometry (SVO) Pipeline

## Overview
This repository contains a C++17 implementation of a Real-Time Stereo Visual Odometry engine. It estimates the 6-DOF (Degrees of Freedom) ego-motion of a moving vehicle using raw stereo camera streams. The pipeline relies on classical geometric computer vision and non-linear optimization, bypassing deep learning dependencies to ensure deterministic execution, high frame rates, and low latency on standard CPU hardware.

## Project Demonstration


https://github.com/user-attachments/assets/4be6ff01-7802-4e48-a274-ec8437f64918



https://github.com/user-attachments/assets/9290b042-16bf-432b-98c2-d27478248837



## System Architecture

The perception pipeline is divided into four sequential modules:

1. **Feature Extraction:** Utilizes the FAST (Features from Accelerated Segment Test) corner detector. Includes Non-Maximum Suppression to isolate high-gradient structural points.
2. **Feature Distribution (Spatial Bucketing):** To mitigate planar degeneracy in the motion solver, the image space is partitioned into a uniform grid. Feature detection is constrained per bucket, enforcing a geometrically diverse distribution of points across the focal plane.
3. **Temporal Tracking:** Implements the Lucas-Kanade Pyramidal Optical Flow (KLT) algorithm to track sub-pixel feature velocities across continuous frames.
4. **Motion Estimation:** Computes the 3D projection of 2D features via stereo baseline triangulation. The final vehicle pose matrix is calculated using a Perspective-n-Point (PnP) solver, optimized via Levenberg-Marquardt, and filtered through a Random Sample Consensus (RANSAC) loop to reject dynamic outliers.

### Architecture & Spatial Bucketing Logic
<img width="2188" height="1390" alt="OpencvP21" src="https://github.com/user-attachments/assets/294c8e56-4e69-4e56-9a5b-4f3131926ee3" />
<img width="2041" height="1597" alt="OpencvP2" src="https://github.com/user-attachments/assets/8bc648df-7f83-499c-8530-cf3067e242fb" />


## Prerequisites
* **OS:** Ubuntu 20.04 / 22.04
* **Compiler:** GCC 9.0+ (C++17 Support required)
* **Build System:** CMake 3.10+
* **Dependencies:** * OpenCV 4.x (Core, Video, Features2D, Calib3d)
  * Eigen3 

## Build Instructions

Clone the repository and compile using CMake:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

```

## Execution

The system provides two executable binaries for validation:

**1. Real-World Optical Flow Validation**
Processes an input video feed to validate the FAST+KLT tracking pipeline and Spatial Bucketing constraints.

```bash
./webcam_live

```

**2. Synthetic Pose Trajectory Mapping**
Validates the geometric PnP solver by simulating a moving camera array and plotting the resulting top-down 2D coordinate trajectory.

```bash
./vo_test

```

## Repository Structure

```text
├── CMakeLists.txt
├── README.md
├── LICENSE
├── .gitignore
├── include/
│   ├── feature_tracker.hpp
│   ├── motion_estimator.hpp
│   └── visual_odometry.hpp
├── src/
│   ├── feature_tracker.cpp
│   ├── motion_estimator.cpp
│   ├── triangulation.cpp
│   └── visual_odometry.cpp
└── tests/
    ├── main.cpp
    └── webcam_live.cpp

```

## Developer Information

* **Name:** Efaj Hossain
* **GitHub:** [ https://github.com/efajbro ]
* **Contact:** info.efajbro@gmail.com

## License

Distributed under the MIT License. See `LICENSE` for more information.


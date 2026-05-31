# Stereo Visual Odometry (SVO) Pipeline



## Overview
This repository contains a production-grade, highly deterministic C++17 implementation of a Real-Time Non-SLAM Stereo Visual Odometry engine. Designed for autonomous robotic platforms operating under strict low-latency constraints, this system continuously estimates the 6-DOF (Degrees of Freedom) ego-motion of a vehicle using calibrated stereo camera streams. By processing sparse feature sets frame-to-frame and modeling measurement uncertainties, the pipeline avoids the intense computational overhead of full SLAM backends (such as global mapping and loop closure) while maintaining high translational and rotational precision.

---

## Theoretical and Mathematical Foundations

The core localization engine relies on rigorous projective geometry, rigid-body kinematics, and non-linear cost minimization techniques. The mathematical formulation of each subsystem is defined below:

### 1. Stereo Projective Geometry & Triangulation
The pipeline operates on rectified stereo frames natively compatible with the KITTI dataset standard, assuming a precisely calibrated horizontal camera baseline distance of $b = 0.54\text{ meters}$. A 3D spatial landmark position $\mathbf{X} = (X, Y, Z)^T$ is mapped to left and right 2D image coordinates via the camera intrinsic matrix $\mathbf{K}$:

$$\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

Given that the stereo image pairs are ideal and rectified, vertical pixel coordinates are perfectly aligned across sensors ($v_l \approx v_r$), and the extrinsic transformation between the frames is defined strictly by a horizontal translation along the camera's X-axis. For any corresponding feature point identified at $(u_l, v)$ in the left camera view and $(u_r, v)$ in the right camera view, the stereo disparity $d$ is given by:

$$d = u_l - u_r$$

Utilizing the computed disparity value, the exact 3D spatial coordinates of the landmark relative to the left camera's optical center are resolved via geometric triangulation:

$$Z = \frac{f_x \cdot b}{d}, \quad X = \frac{(u_l - c_x) \cdot Z}{f_x}, \quad Y = \frac{(v_l - c_y) \cdot Z}{f_y}$$

### 2. Distance-Invariant Rigidity Constraints
To preserve the geometric integrity of tracking before numerical optimization, a structural rigidity check is performed on all temporally matched feature pairs. For any two static 3D landmark coordinates $\mathbf{X}_i^T$ and $\mathbf{X}_j^T$ captured at time $T$, their Euclidean distance must remain perfectly invariant under rigid physical ego-motion. For any pair that violates this spatial consistency condition beyond a strict tuning threshold $\epsilon$, the correspondence is flagged as a dynamic outlier and immediately pruned from the tracking set:

$$\Big| \|\mathbf{X}_i^T - \mathbf{X}_j^T\|_2 - \|\mathbf{X}_i^{T+1} - \mathbf{X}_j^{T+1}\|_2 \Big| \ge \epsilon \implies \text{Reject Outlier}$$

### 3. Non-Linear Pose Estimation (PnP)
The relative camera transformation matrix consisting of rotation $\mathbf{R}_k$ and translation $\mathbf{t}_k$ that projects the 3D world landmarks $\mathbf{X}_i^T$ onto the subsequent 2D tracked coordinates $\mathbf{x}_i^{T+1} = (u, v)_i^{T+1}$ is computed by minimizing the sum of the squared non-linear reprojection errors:

$$\arg\min_{\mathbf{R}_k, \mathbf{t}_k} \sum_{i} \left\| \mathbf{x}_i^{T+1} - \pi\left(\mathbf{K} \left(\mathbf{R}_k \mathbf{X}_i^T + \mathbf{t}_k\right)\right) \right\|_2^2$$

where $\pi(\cdot)$ represents the standard perspective projection function. This highly non-linear system is iteratively solved using the Levenberg-Marquardt optimization algorithm embedded directly within a robust Random Sample Consensus (RANSAC) framework to isolate remaining anomalous feature trajectories.

### 4. 2D-2D Fallback Decomposition & Chirality Checks
For execution frames where active 3D triangulation is unavailable, the relative motion is resolved using 2D-2D feature correspondences to compute the Essential Matrix $\mathbf{E}$. Singular Value Decomposition (SVD) of $\mathbf{E}$ produces four mathematically valid solutions for the camera pose matrix. To resolve this geometric ambiguity, a chirality check is executed: the 3D position of each feature is triangulated across all four candidate pose configurations, and the unique pose that yields strictly positive depth coordinates relative to both camera centers is isolated:

$$Z_{\text{left}} > 0 \quad \text{and} \quad Z_{\text{right}} > 0$$

### 5. Depth Distribution Dynamics
The physical distribution of tracked feature points directly dictates pose estimation errors. The engine continuously tracks point depths against the baseline metric:
* **Near Features ($Z < 21.6\text{ meters}$):** Points with a depth below 40 times the baseline primarily constrain the translation components of the ego-motion solver.
* **Far Features ($Z \ge 21.6\text{ meters}$):** Points with a depth exceeding 40 times the baseline structurally stabilize the rotation components of the ego-motion solver.

### 6. Local Bundle Adjustment & Scale Correction
To bound optimization drift over long trajectories, the pipeline can be configured to execute two optional processing modes:
* **Local Bundle Adjustment:** A sliding window of the $N$ most recent frames is managed via the `g2o` graph optimization library, simultaneously refining camera poses and landmark coordinates to minimize global reprojection errors.
* **Planar Homography Scale Correction:** In environments where a local ground plane is tracked, the inter-image planar homography matrix $\mathbf{H}$ is estimated:

$$\mathbf{H} = \mathbf{R} - \frac{\mathbf{t}\mathbf{n}^T}{h_2}$$

where $\mathbf{n}$ is the unit normal vector of the ground plane and $h_2$ is the camera height relative to the ground. The value of $h_2$ is extracted via SVD of $\mathbf{H}^T\mathbf{H}$ and compared against the known static camera mounting height $h_0$ to generate a scale correction factor:

$$s = \frac{h_0}{h_2}$$

---




## Project Demonstration


https://github.com/user-attachments/assets/4be6ff01-7802-4e48-a274-ec8437f64918



https://github.com/user-attachments/assets/9290b042-16bf-432b-98c2-d27478248837


## System Architecture

The runtime execution pipeline handles streaming video sequences through four highly decoupled modules optimized for low latency:

1. **Feature Detection with Spatial Bucketing:** Features are extracted in the left frame at time $T$ using the FAST corner detector. To guarantee a uniform distribution of points across the focal plane and prevent geometric clustering in high-texture zones, a spatial bucketing grid divides the image into non-overlapping cells. Only a fixed number ($N=10$) of feature points possessing the highest corner response scores are preserved per cell.
2. **Temporal Tracking:** The bucketed features are tracked into the next left frame at time $T+1$ utilizing a Kanade-Lucas-Tomasi (KLT) pyramidal optical flow tracker via `cv::calcOpticalFlowPyrLK`.
3. **Sparse Disparity Computation:** The stereo disparity map is computed for the left frame at time $T$ via sparse Semi-Global Block Matching (`cv::StereoSGBM`), mapping valid feature coordinates into metric 3D space.
4. **Outlier Rejection & Optimization:** Features pass the distance-invariant rigidity constraint loop and are forwarded to the PnP RANSAC optimizer to resolve the final 6-DOF transformation matrix.



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


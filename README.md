## Point Cloud Registration using Custom ICP

This project focuses on rigid point cloud registration using a from-scratch implementation of the Iterative Closest Point (ICP) algorithm. The goal is to align two 3D point clouds by estimating the optimal rigid transformation (rotation and translation) that minimizes the distance between corresponding points.  
Rather than relying on built-in ICP APIs, the project implements the full pipeline manually to demonstrate a deeper understanding of geometric alignment, correspondence estimation, and iterative optimization.  
The project is evaluated in two stages: a controlled indoor environment and a real-world outdoor dataset.

## Approach

A custom ICP pipeline was implemented with the following steps:

1. Nearest-neighbor correspondence search using a KD-tree  
2. Centroid computation and point centering  
3. Rigid transformation estimation using Singular Value Decomposition (SVD)  
4. Iterative refinement of rotation and translation  
5. Convergence checking using Mean Squared Error (MSE)  

At each iteration, the source point cloud is transformed and progressively aligned to the target.

## Evaluation Scenarios

### 1. Open3D Demo Point Clouds

The first evaluation uses structured indoor point clouds provided by Open3D. These point clouds have good overlap and minimal noise, allowing the custom ICP algorithm to converge reliably. The result demonstrates accurate alignment and validates the correctness of the implementation.

**Result:**  
<img width="480" height="362" alt="Screenshot 2026-01-22 at 1 15 18 PM" src="https://github.com/user-attachments/assets/2cddfbfa-45ee-4b2a-9e1a-d11f62f929c9" />


### 2. KITTI LiDAR Dataset

The second evaluation applies the same ICP pipeline to real-world LiDAR scans from the KITTI dataset. These outdoor point clouds are sparse, noisy, and contain dynamic elements such as vehicles.

While partial alignment is achieved, the final registration is less accurate compared to the indoor case. This behavior is expected and highlights known limitations of basic ICP when applied without an initial pose estimate or feature-based matching.

**Result:**  
<img width="623" height="392" alt="Screenshot 2026-01-22 at 1 15 34 PM" src="https://github.com/user-attachments/assets/8ba3a0b0-072b-4f76-bd39-86ef928d27ae" />


## Key Observations

- ICP performs well in structured environments with good initial alignment  
- Performance degrades on real-world outdoor data due to sparsity, noise, and motion  
- Vanilla ICP is sensitive to initialization and can converge to local minima  
- Practical systems often combine ICP with feature extraction, odometry, or sensor fusion  

## How to Run the Code

### Task 1 – Open3D Demo Point Clouds
1. Ensure Python and Open3D are installed.
2. Run the following command:
```bash
python TASK1.py
```
### Task 2b – KITTI LiDAR Point Clouds
1. Ensure the KITTI point cloud files (kitti_frame1.pcd and kitti_frame2.pcd) are available.
2. Update the file paths if necessary.
Run the following command:
```bash
python TASK2.py
```



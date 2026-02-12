# **LAB 1: Kalman Filter / SLAM Implementation Report**

## **Author**
- 65340500058 Anuwit Intet

## **Introduction**
This project aims to study and develop robot localization and mapping procedures, divided into four main parts:
- **Part 0:** Calculating Wheel Odometry to determine the robot's position using the robot's wheel positions.
- **Part 1:** Performing Sensor Fusion between Wheel Odometry and IMU using an Extended Kalman Filter (EKF) to reduce errors caused by wheel rotation and slippage.
- **Part 2:** Position improvement using ICP Scan Matching, using EKF values ​​as the initial guess to generate LiDAR-based odometry.
- **Part 3:** Performing Full SLAM with slam_toolbox to compare the performance of loop closure and complete map generation.

## **Usage**

**How to run this project.**

## **Dataset Description**
The dataset is provided as a ROS bag and contains sensor measurements recorded during robot motion.

Topics included:

- **/scan**: 2D LiDAR laser scans at **5 Hz**
- **/imu**: Gyroscope and accelerometer data at **20 Hz**
- **/joint_states**: Wheel motor position and velocity at **20 Hz**

The dataset is divided into three sequences, each representing a different environmental condition:

**Sequence 00 – Empty Hallway:** A static indoor hallway environment with minimal obstacles and no dynamic objects. This sequence is intended to evaluate baseline odometry and sensor fusion performance.

**Sequence 01 – Non-Empty Hallway with Sharp Turns:** An indoor hallway environment containing obstacles and clutter, with sections of sharp turning motion. This sequence is designed to challenge odometry and scan matching performance under rapid heading changes.

**Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion:** An indoor hallway environment with obstacles, similar to Sequence 2, but recorded with smoother and non-aggressive robot motion. This sequence is intended to evaluate performance under more stable motion conditions.

## **Evaluation**
There are three metrics for comparison of each method such as accuracy, drift, and robustness.

- **Accuracy**
- **Drift**
- **Robustness**

## **Part 0: Calculating Wheel Odometry**

### **Objective**
To establish a baseline reference and study the impact of cumulative drift caused solely by the mechanical system.

### **Theory**

There are two main ways to calculate the odometry of a differential drive robot:

**Type 1: Position-based Odometry (Angular Displacement)**

This method calculates the difference in wheel angular position ($\Delta \phi$) over time, which helps reduce the accumulation of time jitter error in the ROS 2 system.

Calculate wheel distance:

$$\Delta d_{left} = R \cdot (\phi_{L, t} - \phi_{L, t-1})$$

$$\Delta d_{right} = R \cdot (\phi_{R, t} - \phi_{R, t-1})$$

$$\Delta d = \frac{(\Delta d_{right} + \Delta d_{left})}{2}$$

$$\Delta \theta = \frac{(\Delta d_{right} - \Delta d_{left})}{L}$$

Update position:

$$x_{t+1} = x_{t} + \Delta d \cos(\theta_{old} + \frac{\Delta \theta}{2})$$

$$y_{t+1} = y_{t} + \Delta d \sin(\theta_{old} + \frac{\Delta \theta}{2})$$

$$\theta_{t+1} = \theta_{t} + \Delta \theta$$

**Type 2: Velocity-based Odometry (Linear & Angular Velocity)**

This method uses the angular velocity ($\omega_{wheel}$) to calculate the velocity of the robot directly, which is suitable for use in the EKF prediction step as it provides smooth velocity calculations for the robot.

Calculate velocity:

$$v = \frac{R}{2} (\omega_R + \omega_L)$$

$$\omega = \frac{R}{L} (\omega_R - \omega_L)$$

Update position with Integration:

$$x_{t+1} = x_t + v \cos(\theta_t) \Delta t$$

$$y_{t+1} = y_t + v \sin(\theta_t) \Delta t$$

$$\theta_{t+1} = \theta_t + \omega \Delta t$$

### **Setup**

- Data Source: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /joint_states.

- Implementation: Calculate odometry using both Position-based (from wheel position) and Velocity-based (from wheel velocity) methods, using wheel radius (R) = 0.033 meters and wheel base (L) = 0.16 meters to compare the differences between the two methods in all 3 sequences.

- LaserScan (Raw Points) Visualization: Show the structure of the walls that the robot scans throughout the path (Point Cloud Accumulation)

- Path Visualization: Plot the path of movement that is calculated from the sum of the coordinates $(x, y)$ calculated from /joint_states to compare with the scan points

- Usage: You can run this file to see the test results
    - Position_wheel_odom_node.py
    - Velocity_wheel_odom_node.py

- Experiment: Perform Position-based and Velocity-based wheel odometry in all 3 sequences and analyze accuracy, drift, and robustness.

### **Result & Discussion**

``Sequence 00 Empty Hallway``

**Position-based Odometry**

**Velocity-based Odometry**

Analysis
- Accuracy: 
- Drift: 
- Robustness: 

``Sequence 01 Non-Empty Hallway with Sharp Turns``

**Position-based Odometry**

**Velocity-based Odometry**

Analysis
- Accuracy: 
- Drift: 
- Robustness: 

``Sequence 02 Non-Empty Hallway with Non-Aggressive Motion``

**Position-based Odometry**

**Velocity-based Odometry**

Analysis
- Accuracy: 
- Drift: 
- Robustness: 

``Summary``


## **Part 1: Sensor Fusion between Wheel Odometry and IMU using an Extended Kalman Filter (EKF)**

### **Objective**

Develop a state estimator using the Extended Kalman Filter (EKF) to reduce rotational errors that are often highly pronounced in wheel-only systems.

### **Theory**

In this section, we use an EKF to perform sensor fusion between wheel odometry (as the control input) and IMU yaw (as the measurement) to improve positioning accuracy.

**1. Prediction Step (Time Update)**

This step involves predicting the new state of the robot based on the kinematic model:

State Prediction:

$$\hat{x}_{t} = f(\hat{x}_{t-1}, u_t) = \begin{bmatrix} x_{t-1} + v \cos(\psi_{t-1}) \Delta t \\ y_{t-1} + v \sin(\psi_{t-1}) \Delta t \\ \psi_{t-1} + \omega \Delta t \end{bmatrix}$$

Covariance Prediction:

$$P_{t|t-1} = G_t P_{t-1} G_t^T + Q$$

**2. Update Step (Measurement Update)**

This step involves using the measured values from the sensor (IMU) to improve the predicted values:

Innovation (Measurement Residual):

$$y_t = z_t - H \hat{x}_{t}$$

Kalman Gain:

$$K_t = P_{t|t-1} H^T (H P_{t|t-1} H^T + R)^{-1}$$

State & Covariance Update:$$\hat{x}_t = \hat{x}_{t|t-1} + K_t y_t$$

$$P_t = (I - K_t H) P_{t|t-1}$$

**Definition of Variables**

$\hat{x}$ State Vector

$P$ Covariance Matrix - Shows the uncertainty of the robot's state.

$u_t$ Control Input

$G_t$ Jacobian Matrix - Linear estimation matrix of the motion model.

$Q$ Process Noise - Confidence in the model

$z_t$ Measurement - The actual measured value, in this case, is the yaw angle from the IMU.

$H$ Observation Model - Matrix that specifies which sensor measures which state (in this case, only $Yaw$)

$R$ Sensor Noise - Confidence in the sensor

$K_t$ Kalman Gain - Weighting factor between "predicted value" and "actual measured value"

**To determine the position of a two-wheeled robot, each matrix can be defined as follows:**

``State Vector``
$$\hat{x}_{t} = f(\hat{x}_{t-1}, u_t) = \begin{bmatrix} x_{t-1} + v \cos(\psi_{t-1}) \Delta t \\ y_{t-1} + v \sin(\psi_{t-1}) \Delta t \\ \psi_{t-1} + \omega \Delta t \end{bmatrix}$$

Choosing speed ($v, \omega$) instead of position in step prediction attempts to preserve the Markov properties of the system as completely as possible. This is because coordinate positions from odometry are cumulative values ​​that include errors from the past (hidden history). This contradicts the principle that the current state should be sufficient to predict the future. Using instantaneous velocity allows the EKF to assess uncertainty more independently and accurately in practice.

``Jacobian Matrix``

$$G_t = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} \frac{\partial f_x}{\partial x} & \frac{\partial f_x}{\partial y} & \frac{\partial f_x}{\partial \psi} \\ \frac{\partial f_y}{\partial x} & \frac{\partial f_y}{\partial y} & \frac{\partial f_y}{\partial \psi} \\ \frac{\partial f_\psi}{\partial x} & \frac{\partial f_\psi}{\partial y} & \frac{\partial f_\psi}{\partial \psi} \end{bmatrix} = \begin{bmatrix} 1 & 0 & -v \sin(\psi) \Delta t \\ 0 & 1 & v \cos(\psi) \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

``Process Noise Matrix``
$$Q = \begin{bmatrix} \sigma_x^2 & 0 & 0 \\ 0 & \sigma_y^2 & 0 \\ 0 & 0 & \sigma_\psi^2 \end{bmatrix}$$

- Model reliability coefficient
    - If the value is low, the system trusts the model more.
    - If the value is high, the system trusts the IMU more.

``Sensor Noise Matrix``
$$R = [\sigma_{imu\_yaw}^2]$$

- $\sigma_{imu\_yaw}^2$: The variance of the IMU signal. 
    - **Small value:** The system will trust the IMU more (the path will be straighter, but may wobble due to sensor noise). 
    - **Large value:** The system will trust it less (the path will be smooth, but will accumulate drift easily).

``Initial State Covariance Matrix``
$$P_0 = \begin{bmatrix} p_{xx} & 0 & 0 \\ 0 & p_{yy} & 0 \\ 0 & 0 & p_{\psi\psi} \end{bmatrix}$$

``Observation Model Matrix``
$$H = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}$$

In this case, the observation model is only $Yaw$

``Measurement``

In this case, the measurement is the yaw angle from the IMU.

### **Setup**

- Data Source: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /joint_states and /imu.

- Implementation: Calculate wheel odometry using Velocity-based (from wheel velocity) methods as a initial guess, using wheel radius (R) = 0.033 meters and wheel base (L) = 0.16 meters. And perform EKF to improve the accuracy of the wheel odometry.

- LaserScan (Raw Points) Visualization: Show the structure of the walls that the robot scans throughout the path (Point Cloud Accumulation) on rviz2.

- Path Visualization: Plot the path of movement from EKF in each timestep on rviz2.

- Usage: You can run this file to see the test results
    - EKF_odom_node.py

- Experiment: Perform EKF in variance of $Q$ and $R$ in all 3 sequences and analyze accuracy, drift, and robustness.
    - Experiment 1: $Q$ = 0.000001, 0.001, 1.0, 1000.0 and $R$ = 0.1
    - Experiment 2: $R$ = 0.0001, 0.1, 1000.0, 100000.0 and $Q$ = 0.001
    - Choose the best $Q$ and $R$ and compare the results to Position-based and Velocity-based odometry.

### **Result & Discussion**

## **Part 2: Position improvement using ICP Scan Matching**

### **Objective**

To refine the EKF-based odometry using LiDAR scan matching and evaluate the improvement in accuracy and drift.

### **Theory**

The Iterative Closest Point (ICP) algorithm is a fundamental technique for geometric registration. It refines the robot's pose by iteratively aligning a current LiDAR scan (Source) with a reference model (Target), such as a previous scan or a global map. The core objective is to minimize a defined error metric between these two data sets to achieve decimeter-level localization accuracy.

Depending on the environment's structure, different error metrics can be employed:

1. **Point-to-Point ICP**: This is the "classic" version of ICP. It treats the LiDAR data as a simple set of coordinates without considering the geometry of the environment.

- Logic: It calculates the Euclidean distance directly between source point $p_i$ and the nearest target point $q_i$.

- Error Function: $E = \sum || p_i - q_i ||^2$

- Characteristics:

    - Pros: Minimal computation per iteration; easy to implement.

    - Cons: Struggles with "sliding" along flat walls. If the robot moves parallel to a wall, the points may pull toward each other incorrectly, leading to longitudinal drift.

2. **Point-to-Plane ICP**: This version is more "geometry-aware" and is the standard for indoor mobile robotics where flat surfaces (walls) are prevalent.

- Logic: Instead of pulling a point toward another point, it pulls the point toward the tangent plane (or line in 2D) of the target surface.

- Error Function: $E = \sum ((p_i - q_i) \cdot n_i)^2$ (where $n_i$ is the surface normal).

- Characteristics:

    - Pros: Much faster convergence. It allows points to "slide" along the wall as long as they stay on the same plane, which is exactly how LiDAR scans behave on long corridors.

    - Cons: Requires calculating surface normals for every point, which adds a slight initial computational cost.

**ICP steps**

1. Point Cloud Pre-processing:
    - Convert raw LaserScan data (ranges and angles) into 2D Cartesian coordinates $(x, y)$.
    - **Downsampling**: Pick every $n^{th}$ point (e.g., every 5th point) to reduce computational load while maintaining structural features.

2. Initial Guess:
    - Apply the current estimated pose from Odometry/EKF to the current scan to bring it close to the target scan.

3. Nearest Neighbor Association:
    - For each point in the current scan, find the closest point in the previous scan.
    - Implementation Note: We use KDTree for efficient spatial searching, reducing complexity from $O(N^2)$ to $O(N \log N)$.
    - When transitioning to Point-to-Plane, the algorithm not only finds the nearest point but also estimates the surface normal of the local neighborhood.

4. Motion Estimation:
    - Calculate the Singular Value Decomposition (SVD) to find the optimal Rotation ($R$) and Translation ($T$) that minimizes the Mean Squared Error (MSE) between the paired points. For Point-to-Plane, we transition to a Non-linear Optimizer to account for surface normals, providing better stability in structured environments

5. Transformation Update:
    - Apply the calculated $R$ and $T$ to the current point cloud.

6. Iteration & Convergence:
    - Repeat steps 3–5 until the change in error is below a threshold (EPS) or the maximum number of iterations (MAX_ITER) is reached.

This implementation can be enhanced with Outlier Rejection using distance thresholds and Initial Guess Integration from EKF. These techniques ensure the ICP remains robust even in dynamic environments with moving obstacles.

**Outlier Rejection Strategies in ICP**

To ensure the robustness of our SLAM system, especially in dynamic or noisy environments, we implement a Multi-stage Outlier Rejection pipeline. This prevents "bad data" (like moving people or sensor noise) from distorting the robot's localization.

1. **Pre-Filtering (Raw Data Cleaning)**: Filters out noise and reduces computational load before the matching process begins.
    - Voxel Grid Filtering:
        - Method: Divides the 2D space into small grids (Voxels) and replaces all points within a grid with their centroid.
        - Benefit: Manages High-Density areas where overlapping points could disproportionately bias the optimization. It ensures a uniform distribution of data.
    - Statistical Outlier Removal (SOR):
        - Method: Calculates the mean distance of each point to its $k$-nearest neighbors. Points exceeding a global standard deviation threshold are discarded.
        - Benefit: Effectively removes sparse noise such as dust, steam, or sensor "ghost" points.

2. **Correspondence Rejection (Post-Matching Filter)**: Filters point pairs after the Nearest Neighbor search but before calculating the transformation ($R, T$).
    - Distance Thresholding:
        - Method: Rejects point pairs if the distance between the source and target exceeds a set limit (e.g., 0.5m).
        - Rationale: Large distances often indicate Dynamic Objects (e.g., a person walking past) that do not exist in the reference map.
    - Reciprocal Correspondence:
        - Method: Validates pairs bi-directionally. Point $A$ (Source) must find Point $B$ (Target) as its closest neighbor, AND Point $B$ must also identify Point $A$ as its closest neighbor.
        - Benefit: Eliminates "ambiguous" matches, particularly in featureless environments where a point might otherwise snap to a random distant surface.

3. **Robust Estimators (Optimization-Level Filtering)**: The final layer of defense, integrated into the Step 4: Motion Estimation phase using advanced mathematics.
    - Huber Loss Function:
        - Concept: A hybrid loss function that adjusts how "Error" is weighted.
            - Small Errors (Inliers): Treated quadratically ($e^2$) for high precision.
            - Large Errors (Outliers): Treated linearly ($|e|$) to limit their influence.
        - Outcome: Prevents Position Jumps. Even if an outlier persists, it cannot exert enough "pull" to significantly shift the robot's estimated pose.

### **Setup** 

Not Finish

```bash
- Data Source: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /joint_states and /imu for EKF-based odometry and /scan for ICP-based odometry.

- Implementation: Perform EKF-based odometry as a initial guess for ICP-based odometry.

- LaserScan (Raw Points) Visualization: Show the structure of the walls that the robot scans throughout the path (Point Cloud Accumulation) on rviz2.

- Path Visualization: Plot the path of movement from EKF in each timestep on rviz2.

- Usage: You can run this file to see the test results
    - EKF_odom_node.py

- Experiment: Perform EKF in variance of $Q$ and $R$ in all 3 sequences and analyze accuracy, drift, and robustness.
    - Experiment 1: $Q$ = 0.000001, 0.001, 1.0, 1000.0 and $R$ = 0.1
    - Experiment 2: $R$ = 0.0001, 0.1, 1000.0, 100000.0 and $Q$ = 0.001
    - Choose the best $Q$ and $R$ and compare the results to Position-based and Velocity-based odometry.
```
### **Result & Discussion**

## **Part 3: Full SLAM with slam_toolbox**

### **Objective**

To perform full SLAM using slam_toolbox and compare its pose estimation and mapping performance with the ICP-based odometry from Part 2.

### **Theory**

This part utilizes Slam Toolbox, a powerful 2D SLAM framework developed by Steve Macenski. It provides a comprehensive set of tools for mapping and localization, outperforming many free and commercial alternatives.

**Key Features**

- **Standard 2D SLAM**: Supports the "point-and-shoot" mapping workflow (start, map, and save .pgm files) with built-in utilities.

- **Lifelong Mapping**: Ability to load a saved pose-graph and continue mapping while automatically removing extraneous or redundant information.

- **Pose-Graph Refinement**: Refine, remap, or continue mapping from a serialized pose-graph at any time.

- **Advanced Localization**: Features an optimization-based localization mode. It can also run in "LiDAR Odometry" mode without a prior map using local loop closures.

- **Dual Processing Modes**: Supports both Synchronous and Asynchronous mapping modes to balance between processing all scans and maintaining real-time performance.

- **Ceres Optimizer**: Powered by a new optimized plugin based on Google Ceres for high-performance graph optimization.

- **Interactive Tools**: Includes an RVIZ plugin for direct interaction, allowing manual manipulation of nodes and graph connections.

**Parameters**


### **Setup**

- Data Source: Use data from the ROS bag in all 3 sequences (00, 01, 02), focusing on data from Topic /scan for send to slam_toolbox.

- Implementation: 
    - Utilize the Synchronous mode to ensure that every single LaserScan message is processed and integrated into the pose-graph.
    - Use XXX as a odometry source for slam_toolbox.

- Map Visualization: Show the map from slam_toolbox using topic /map on rviz2.

- Path Visualization: Plot the path of movement from slam_toolbox on rviz2.

- Usage: You can run this file to see the test results
    - SLAM_node.py

- Experiment: 

- Analysis: Accuracy, Drift, Robustness

### **Result & Discussion**

## **Conclusion & Discussion**

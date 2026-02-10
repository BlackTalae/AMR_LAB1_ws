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

/scan: 2D LiDAR laser scans at 5 Hz
/imu: Gyroscope and accelerometer data at 20 Hz
/joint_states: Wheel motor position and velocity at 20 Hz
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

- Analysis: Accuracy, Drift, Robustness

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

### **Result & Discussion**

## **Part 2: Position improvement using ICP Scan Matching**

### **Objective**

### **Theory**

### **Experiment**

### **Result & Discussion**

## **Part 3: Full SLAM with slam_toolbox**

### **Objective**

### **Theory**

### **Experiment**

### **Result & Discussion**

## **Conclusion & Discussion**

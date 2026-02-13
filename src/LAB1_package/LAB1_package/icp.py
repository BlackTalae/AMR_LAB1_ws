#!/usr/bin/python3

import numpy as np
from scipy.spatial import KDTree


def scan_to_pointcloud(LaserScanMsg):
    """
    Convert LaserScan message to 2D point cloud
    
    Args:
        LaserScanMsg: ROS LaserScan message
    
    Returns:
        point_cloud: numpy array of shape (N, 2) with x, y coordinates
    """
    ranges = np.array(LaserScanMsg.ranges)
    angles = LaserScanMsg.angle_min + np.arange(len(ranges)) * LaserScanMsg.angle_increment
    
    valid_indices = (ranges >= LaserScanMsg.range_min) & (ranges <= LaserScanMsg.range_max)
    r = ranges[valid_indices]
    phi = angles[valid_indices]
    
    # Convert Polar to Cartesian coordinates (x, y, z)
    x = r * np.cos(phi)
    y = r * np.sin(phi)
    z = np.zeros_like(x) # 2D scan usually has 0 for height

    # point_cloud = np.vstack((x, y, z))
    point_cloud = np.stack((x, y),axis=0).T
    # print(point_cloud.shape)
    
    return point_cloud


def pointcloud_to_odom(point_cloud, x, y, theta):
    """
    Transform point cloud from base_link frame to odom frame
    
    Args:
        point_cloud: numpy array of shape (N, 2) in base_link frame
        x, y, theta: robot pose in odom frame
    
    Returns:
        transformed point cloud in odom frame, shape (N, 2)
    """
    X = point_cloud[:, 0]
    Y = point_cloud[:, 1]

    c = np.cos(theta)
    s = np.sin(theta)

    Xw = c * X - s * Y + x
    Yw = s * X + c * Y + y
    return np.column_stack((Xw, Yw))   # shape (N, 2)


def v2T(dx, dy, dyaw):
    """
    Convert pose vector to transformation matrix
    
    Args:
        dx, dy, dyaw: translation and rotation
    
    Returns:
        3x3 transformation matrix
    """
    c, s = np.cos(dyaw), np.sin(dyaw)
    return np.array([[c, -s, dx],
                    [s,  c, dy],
                    [0,  0, 1]], dtype=float)


def voxel_grid_filter(points, leaf_size=0.05):
    """ 
    Downsample point cloud using voxel grid filter
    
    Args:
        points: numpy array (N, 2) 
        leaf_size: voxel size (m)
    
    Returns:
        filtered points
    """
    if points.shape[0] == 0:
        return points

    grid_pts = np.round(points / leaf_size) * leaf_size
    
    _, unique_indices = np.unique(grid_pts, axis=0, return_index=True)
    
    return points[unique_indices, :] # [n , 2]


def remove_outliers(points, radius=0.2, min_neighbors=5):
    """
    Outlier Removal using radius-based filtering
    
    Args:
        points: numpy array (N, 2)
        radius: radius for neighbor search (meters)
        min_neighbors: minimum number of neighbors required
    
    Returns:
        filtered points with outliers removed
    """
    if points.shape[0] < min_neighbors:
        return points # จุดน้อยเกิน ไม่ต้องกรอง

    # 1. สร้าง KDTree เพื่อค้นหาเพื่อนบ้านอย่างรวดเร็ว
    # (ต้อง Transpose เป็น (N, 2) เพื่อเข้า KDTree)
    tree = KDTree(points) 
    
    # 2. นับจำนวนเพื่อนในรัศมีที่กำหนด
    # query_ball_point จะคืน list ของ index เพื่อนบ้าน
    # return_length=True จะคืนจำนวนเพื่อนมาให้เลย (เร็วมาก)
    neighbors_count = tree.query_ball_point(points, r=radius, return_length=True)
    neighbors_count = np.array(neighbors_count)
    
    # 3. เก็บเฉพาะจุดที่มีเพื่อนมากกว่าที่กำหนด
    # (ลบตัวเองออก 1 เสียง เพราะ KDTree นับตัวเองเป็นเพื่อนด้วย)
    mask = neighbors_count > min_neighbors 
    
    return points[mask, :]


def wrap(a):
    """
    Wrap angle to [-pi, pi]
    
    Args:
        a: angle in radians
    
    Returns:
        wrapped angle in [-pi, pi]
    """
    return np.arctan2(np.sin(a), np.cos(a))

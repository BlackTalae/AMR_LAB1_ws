import numpy as np
from scipy.spatial import KDTree


def data_association(previous_points, current_points, max_corr_dist=None):
    """
    Find nearest neighbors between point clouds
    
    Args:
        previous_points: Reference point cloud (N, 2)
        current_points: Query point cloud (M, 2)
        max_corr_dist: Maximum correspondence distance (optional)
    
    Returns:
        current_matched: Matched points from current
        previous_matched: Corresponding points from previous
        distances: Distances between matches
        indices: Indices of matches in previous_points
    """
    tree = KDTree(previous_points)
    dists, idx = tree.query(current_points, k=1)
    previous_points_match = previous_points[idx, :]

    if max_corr_dist is not None:
        mask = dists < max_corr_dist
        return current_points[mask], previous_points_match[mask], dists[mask], idx[mask]
    
    return current_points, previous_points_match, dists, idx


def icp_matching(previous_points, current_points, 
                init_x=None, init_y=None, init_theta=None,
                MAX_ITERATION=30, ERROR_BOUND=0.001, max_corr_dist=None):
    """
    Iterative Closest Point (ICP) for 2D point cloud alignment
    
    Args:
        previous_points: Reference point cloud (target) in map/odom frame, shape (N, 2)
        current_points: Query point cloud (source) in base_link frame OR already transformed, shape (M, 2)
        init_x, init_y, init_theta: Initial guess for transformation (optional)
        MAX_ITERATION: Maximum number of iterations
        ERROR_BOUND: Convergence threshold for error reduction
        max_corr_dist: Maximum distance for point correspondences (optional)
    
    Returns:
        x, y, theta: Final transformation (absolute pose in map frame)
        count: Number of iterations
        error: Final mean error
        success: Whether ICP converged successfully
    
    Notes:
        - If init_x/y/theta are None, assumes current_points is already in map frame
        - If init_x/y/theta are provided, applies initial transformation first
        - Returns ABSOLUTE pose, not incremental transform
    """
    
    # Copy to avoid modifying input
    prev_pts = previous_points.copy()
    curr_pts = current_points.copy()
    
    # Initialize transformation
    if init_x is not None and init_y is not None and init_theta is not None:
        # Apply initial guess ONCE at the beginning
        c, s = np.cos(init_theta), np.sin(init_theta)
        R_init = np.array([[c, -s], [s, c]])
        t_init = np.array([init_x, init_y])
        
        # Transform current points to initial pose
        curr_pts = (R_init @ curr_pts.T).T + t_init
        
        # Start with initial transformation
        R_total = R_init.copy()
        t_total = t_init.copy()
    else:
        # No initial guess - assume current_points already transformed
        R_total = np.eye(2)
        t_total = np.array([0.0, 0.0])
    
    prevError = np.inf
    count = 0
    
    for iteration in range(MAX_ITERATION):
        count += 1
        
        # 1) Find correspondences
        src, tgt, dists, indices = data_association(prev_pts, curr_pts, max_corr_dist=max_corr_dist)
        
        # Check if enough correspondences
        if src.shape[0] < 3:
            # Not enough matches
            success = False
            error = prevError
            break
        

        # 2) Calculate error
        keep = int(0.8 * len(dists))
        idx_sort = np.argsort(dists)
        sel = idx_sort[:keep]
        src = src[sel]; tgt = tgt[sel]; dists = dists[sel]
        error = np.mean(dists)
        
        # 3) Check convergence
        if abs(prevError - error) < ERROR_BOUND:
            success = True
            break
        
        prevError = error
        
        # 4) Compute transformation using SVD (Kabsch algorithm)
        src_centroid = src.mean(axis=0)
        tgt_centroid = tgt.mean(axis=0)
        
        # Center the points
        src_centered = src - src_centroid
        tgt_centered = tgt - tgt_centroid
        
        # Compute cross-covariance matrix
        H = src_centered.T @ tgt_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        
        # Compute rotation (with reflection handling)
        d = np.linalg.det(Vt.T @ U.T)
        R_inc = Vt.T @ np.diag([1.0, np.sign(d)]) @ U.T
        
        # Compute translation
        t_inc = tgt_centroid - R_inc @ src_centroid
        
        # 5) Apply incremental transformation to current points
        curr_pts = (R_inc @ curr_pts.T).T + t_inc
        
        # 6) Accumulate total transformation
        R_total = R_inc @ R_total
        t_total = R_inc @ t_total + t_inc
    
    # Check if we reached max iterations without converging
    if count >= MAX_ITERATION:
        success = error < ERROR_BOUND
    
    # Extract final pose
    x = t_total[0]
    y = t_total[1]
    theta = np.arctan2(R_total[1, 0], R_total[0, 0])
    
    return x, y, theta, count, error, success

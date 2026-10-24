# VRai 
#5/01/26

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from painting_robot_msgs.msg import WallPose, WallStatus
import numpy as np
import math
import random
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ── Classification targets (robot frame, normal pointing toward robot) ────────
RIGHT_WALL_THETA = math.pi / 2    # +90° — right wall normal points +Y
FRONT_WALL_THETA = math.pi        # ±180° — front wall normal points -X

# ── Tuning constants ──────────────────────────────────────────────────────────
RANSAC_ITERATIONS    = 100
RANSAC_THRESHOLD     = 0.05        # m — point-to-line inlier distance
RANSAC_MIN_INLIERS   = 8           # minimum inliers for a valid line
MIN_WALL_LENGTH      = 0.3         # m — reject very short detections
CLASSIFY_ANGLE_TOL   = math.radians(30.0)   # classification tolerance
ASSOC_ANGLE_THRESH   = math.radians(20.0)   # temporal association angle gate
ASSOC_DIST_THRESH    = 0.3         # m — temporal association distance gate
EMA_ALPHA            = 0.3         # smoothing — lower = smoother/slower
MAX_MISSED_FRAMES    = 5           # frames before invalidating lost wall
# ── LiDAR mounting ────────────────────────────────────────────────────────────
LIDAR_YAW_OFFSET     =  -np.pi      # Rotates scan so arrow→code 0° (front), right wall→270°.
                               

def angle_diff(a, b):
    """Signed angular difference wrapped to [-π, π]."""
    d = a - b
    return (d + math.pi) % (2 * math.pi) - math.pi
# ── Constants ─────────────────────────────────────────────────────────────────
FRONT_CENTER_DEG  = 0.0
FRONT_SPREAD_DEG  = 30.0
RIGHT_CENTER_DEG  = 275.0     # default fallback; overridden by 'right_center_deg' param
RIGHT_SPREAD_DEG  = 15.0      # default fallback; overridden by 'right_spread_deg' param (=> 260-290)
FRONT_TARGET_DEG  = 180.0
RIGHT_TARGET_DEG  = 90.0
THETA_TOL_DEG     = 30.0
RANGE_TOL         = 0.25      # 25% relative tolerance
MIN_RANGE         = 0.15
MAX_RANGE         = 15.0
RIGHT_MAX_DIST    = 1.75      # default fallback; overridden by 'right_max_fit_dist' param
FRONT_MAX_DIST    = 12.0

# @todo move all this to geometry utils and env monitor node
def ransac_line(points, n_iter=100, threshold=0.05, min_inliers=8):
    """
    RANSAC line fit on 2D point cloud.
    
    Returns:
        best_mask  : boolean array, True = inlier
        best_n     : normal vector (unit, pointing toward origin)
        best_d     : perpendicular distance from origin to line
        best_count : number of inliers
    Or None if no valid line found.
    """
    n_pts = len(points)
    if n_pts < 2:
        return None

    best_mask  = None
    best_count = 0
    best_n     = None
    best_d     = 0.0

    for _ in range(n_iter):
        # 1. Sample 2 random points
        idx = random.sample(range(n_pts), 2)
        p1, p2 = points[idx[0]], points[idx[1]]

        # 2. Compute line normal
        direction = p2 - p1
        seg_len   = np.linalg.norm(direction)
        if seg_len < 1e-6:
            continue

        n = np.array([-direction[1], direction[0]]) / seg_len
        d = float(np.dot(n, p1))

        # Keep d positive (temp convention for scoring only)
        if d < 0:
            n = -n
            d = -d

        # 3. Count inliers
        dists = np.abs(points @ n - d)
        mask  = dists < threshold
        count = int(mask.sum())

        if count > best_count:
            best_count = count
            best_mask  = mask.copy()
            best_n     = n.copy()
            best_d     = d

    if best_mask is None or best_count < min_inliers:
        return None

    return best_mask, best_n, best_d, best_count

def arc_slice(points, angles_rad, center_deg, spread_deg):
        """
        Returns boolean mask of points within the arc.
        center_deg and spread_deg in degrees.
        Handles wrap-around (e.g. arc crossing 0°/360°).
        """
        center_rad = np.radians(center_deg)
        spread_rad = np.radians(spread_deg)

        # Wrap difference to [-pi, pi]
        diff = (angles_rad - center_rad + np.pi) % (2 * np.pi) - np.pi
        return np.abs(diff) <= spread_rad




def pca_fit(arc_points):
        """
        Fit a line to arc_points using PCA.
        Returns (normal, distance, centroid, eigenvalues) or None if too few points.
        """
        if len(arc_points) < 5:
            return None

        cx, cy = arc_points.mean(axis=0)
        centroid = np.array([cx, cy])

        # Center the points
        centered = arc_points - centroid

        # Covariance matrix
        cov = (centered.T @ centered) / len(arc_points)

        # Eigendecomposition — eigh returns sorted ascending
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # smallest eigenvector = normal direction (perpendicular to wall)
        normal = eigenvectors[:, 0]

        # Flip normal to point toward robot at origin
        # If normal points same way as centroid-from-origin, flip it
        if np.dot(normal, centroid) > 0:
            normal = -normal

        # Distance from origin to line
        distance = float(abs(np.dot(normal, centroid)))

        ratio = eigenvalues[1] / (eigenvalues[0] + 1e-9)

        return normal, distance, centroid, eigenvalues, ratio

def range_check(ranges, angles_deg_array, center_deg, window_deg=5,
                max_dist=None, fitted_dist=None, rel_tolerance=0.25):
    """
    Median range check at a key angle with a ±window_deg sweep.
    
    center_deg    : angle to check (0=front, 270=right)
    max_dist      : hard max distance threshold (m)
    fitted_dist   : distance from PCA fit — used for relative check
    rel_tolerance : reject if median differs from fitted_dist by more than this fraction
    
    Returns (median_range, passed: bool, reason: str)
    """
    # build index array from angle array
    diff = np.abs((angles_deg_array - center_deg + 180) % 360 - 180)
    window_mask = diff <= window_deg
    window_ranges = ranges[window_mask]
    
    # filter valid only
    valid = np.isfinite(window_ranges) & (window_ranges > 0.05)
    if valid.sum() < 3:
        return None, False, f'insufficient rays at {center_deg}°'
    
    median_r = float(np.median(window_ranges[valid]))
    
    # hard max check
    if max_dist is not None and median_r > max_dist:
        return median_r, False, f'median {median_r:.2f}m > max {max_dist:.2f}m'
    
    # relative check against fitted distance
    if fitted_dist is not None:
        rel_err = abs(median_r - fitted_dist) / (fitted_dist + 1e-6)
        if rel_err > rel_tolerance:
            return median_r, False, f'median {median_r:.2f}m vs fitted {fitted_dist:.2f}m ({rel_err*100:.0f}% diff)'
    
    return median_r, True, 'ok'

def theta_check(pca_result, target_deg, tolerance_deg=30):
    """
    Validate fitted normal direction against expected target angle.
    
    target_deg    : expected normal direction (180=front, 90=right)
    tolerance_deg : max deviation allowed
    
    Returns (theta_deg, passed: bool, reason: str)
    """
    if pca_result is None:
        return None, False, 'no result'
    
    n, d, c, eigs, ratio = pca_result
    theta = np.degrees(np.arctan2(n[1], n[0]))
    
    # wrapped angular difference
    diff = abs((theta - target_deg + 180) % 360 - 180)
    
    if diff > tolerance_deg:
        return theta, False, f'theta={theta:.1f}° diff={diff:.1f}° > tol={tolerance_deg}°'
    
    return theta, True, f'theta={theta:.1f}° diff={diff:.1f}° ok'


class WallDetectionNode(Node):

    def __init__(self):
        super().__init__('wall_detection_node')

        # Parameters
        self.declare_parameter('min_range',    0.15)
        self.declare_parameter('max_range',    15.0)
        self.declare_parameter('publish_rate', 10.0)
        # Alignment params — shared with wall_motion via common_alignment_params.yaml
        self.declare_parameter('target_distance', 0.549)
        self.declare_parameter('dist_tolerance',  0.1)
        self.declare_parameter('angle_tolerance', 0.02)
        self.declare_parameter('lidar_offset_x', 0.195)
        self.declare_parameter('lidar_offset_y', -0.147)
        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tol    = self.get_parameter('dist_tolerance').value
        self.angle_tol   = self.get_parameter('angle_tolerance').value
    
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y = self.get_parameter('lidar_offset_y').value

        # Right-wall arc + near-band fit filter (tunable from YAML). Defaults mirror
        # the module constants. right_max_fit_dist caps the RIGHT-arc points fed to
        # RANSAC so a far background wall can't outvote a near narrow column / suck-in.
        self.declare_parameter('right_center_deg',   RIGHT_CENTER_DEG)
        self.declare_parameter('right_spread_deg',   RIGHT_SPREAD_DEG)
        self.declare_parameter('right_max_fit_dist', RIGHT_MAX_DIST)
        self.right_center_deg   = self.get_parameter('right_center_deg').value
        self.right_spread_deg   = self.get_parameter('right_spread_deg').value
        self.right_max_fit_dist = self.get_parameter('right_max_fit_dist').value

        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # Scan state
        self.scan = None

        # FSM state
        self.robot_state = 'IDLE'

        # EMA low pass filter 
        self._right_state = None   # (d_ema, theta_ema)
        self._front_state = None
        self._right_missed = 0
        self._front_missed = 0

        self.EMA_ALPHA  = 0.3
        self.MAX_MISSED = 5

        # Subscriptions
        self.create_subscription(LaserScan, '/scan',        self._scan_cb,  10)
        self.create_subscription(String,    '/robot_state', self._state_cb, 10)

        # Publishers
        self._right_pub  = self.create_publisher(WallPose,   '/right_wall_pose', 10)
        self._front_pub  = self.create_publisher(WallPose,   '/front_wall_pose', 10)
        self._status_pub = self.create_publisher(WallStatus, '/wall_status',      10)
        self._marker_pub = self.create_publisher(MarkerArray, '/wall_markers', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._detection_loop)

        self.get_logger().info('WallDetectionNode started')

    # ── Callbacks ──────────────────────────────────────────────────────────────
        
    def _scan_cb(self, msg):
        self.scan = {
            'ranges'          : list(msg.ranges),
            'angle_min'       : msg.angle_min,
            'angle_increment' : msg.angle_increment,
        }    

    def _state_cb(self, msg):
        self.robot_state = msg.data

    # ── Main detection loop ────────────────────────────────────────────────────

    def _detection_loop(self):
        if self.scan is None:
            return

        # 1. Run full detection pipeline on current scan frame
        results = self._fit_walls_in_frame(self.scan)

        def _fmt(res):
            if res is None:
                return 'none'
            _, d, theta, _ = res
            return f'd={d:.2f}m theta={math.degrees(theta) % 360:.0f}deg'
        self.get_logger().debug(
            f'[wall] right: {_fmt(results["right"])} | front: {_fmt(results["front"])}',
            throttle_duration_sec=2.0)
        

        # right = results['right']   # (score, d, theta, n) or None
        # front = results['front']   # (score, d, theta, n) or None

        #  EMA filtering
        right_filtered = self._update_wall('right', results['right'])
        front_filtered = self._update_wall('front', results['front'])

        # 3. Publish
        self._publish_wall(right_filtered, self._right_pub, is_right=True)
        self._publish_wall(front_filtered, self._front_pub, is_right=False)
        self._publish_markers(results['right'], results['front'])




    #_________ 
    def _fit_walls_in_frame(self, scan_data):        
        """
        Takes one scan frame dict with keys: ranges, angle_min, angle_increment.
        Pipeline per wall:
            1. filter valid points
            2. arc slice
            3. RANSAC → inliers
            4. PCA on inliers
            5. range check
            6. theta check
        Returns:
            dict with keys 'right' and 'front'.
            Each value is (score, d, theta, n) or None if detection failed.
            - score : eigenvalue ratio (quality proxy)
            - d     : perpendicular distance from origin to wall (metres)
            - theta : raw normal angle in radians (atan2(ny, nx))
            - n     : unit normal vector np.array([nx, ny])
        """
        # ── 1. Filter valid points ─────────────────────────────────────────────
        log = self.get_logger()
        
        ranges    = np.array(scan_data['ranges'], dtype=np.float32)
        angle_min = scan_data['angle_min']
        angle_inc = scan_data['angle_increment']
       

        angles    = (angle_min + np.arange(len(ranges)) * angle_inc
                     + LIDAR_YAW_OFFSET) % (2 * np.pi)

        valid   = np.isfinite(ranges) & (ranges > MIN_RANGE) & (ranges < MAX_RANGE)
        r       = ranges[valid]
        a       = angles[valid]
        points  = np.column_stack([r * np.cos(a), r * np.sin(a)])

        angles_deg_full = np.degrees(angles)
        # ── 2. Arc slice ───────────────────────────────────────────────────────
        front_mask = arc_slice(points, a, center_deg=FRONT_CENTER_DEG,
                            spread_deg=FRONT_SPREAD_DEG)
        right_mask = arc_slice(points, a, center_deg=self.right_center_deg,
                            spread_deg=self.right_spread_deg)

        front_pts = points[front_mask]
        # Pre-RANSAC near-band filter on the RIGHT arc only: drop points beyond
        # right_max_fit_dist so RANSAC can't lock onto a far background wall when a
        # near narrow column / suck-in is the real right wall. The post-fit
        # range_check still uses RIGHT_MAX_DIST on the median; this applies the same
        # cap to the fit INPUT. Revert by setting right_max_fit_dist large.
        right_pts = points[right_mask & (np.linalg.norm(points, axis=1) < self.right_max_fit_dist)]

        self.get_logger().debug(
            f'[dbg] valid_pts={len(points)} right_arc={len(right_pts)} front_arc={len(front_pts)}',
            throttle_duration_sec=2.0)

        # ── 3+4. RANSAC → PCA per wall ────────────────────────────────────────
        def ransac_then_pca(pts):
            result = ransac_line(pts)
            if result is None:
                return None
            mask, _, _, _ = result
            return pca_fit(pts[mask])

        front_result = ransac_then_pca(front_pts)
        right_result = ransac_then_pca(right_pts)

        # ── 5. Range check ────────────────────────────────────────────────────
        def apply_range_check(result, center_deg, max_dist, label):
            if result is None:
                return None
            _, d, _, _, _ = result
            median_r, passed, reason = range_check(
                ranges, angles_deg_full,
                center_deg=center_deg,
                max_dist=max_dist,
                fitted_dist=d,
                rel_tolerance=RANGE_TOL
            )
            # print(f"{label} range check: {reason}  median={median_r:.2f}m")
            return result if passed else None

        front_result = apply_range_check(front_result, FRONT_CENTER_DEG,
                                        FRONT_MAX_DIST, 'Front')
        right_result = apply_range_check(right_result, self.right_center_deg,
                                        self.right_max_fit_dist, 'Right')

        # ── 6. Theta check ────────────────────────────────────────────────────
        def apply_theta_check(result, target_deg, label):
            if result is None:
                return None
            theta, passed, reason = theta_check(result,
                                                target_deg=target_deg,
                                                tolerance_deg=THETA_TOL_DEG)
            # print(f"{label} theta check: {reason}")
            return result if passed else None

        front_result = apply_theta_check(front_result, FRONT_TARGET_DEG, 'Front')
        right_result = apply_theta_check(right_result, RIGHT_TARGET_DEG, 'Right')

        # ── 7. Pack return value ──────────────────────────────────────────────
        def pack(result):
            if result is None:
                return None
            n, d, c, eigs, ratio = result
            theta = float(np.arctan2(n[1], n[0]))
            score = ratio                          # quality proxy for EMA tracker
            return (score, d, theta, n)

        # debug — suppress after tuning
        for label, res in [('Right', right_result), ('Front', front_result)]:
            if res is not None:
                n, d, c, eigs, ratio = res
                self.get_logger().debug(
                    f'{label} eigenvalue ratio={ratio:.1f}',
                    throttle_duration_sec=5.0)
        return {
            'right': pack(right_result),
            'front': pack(front_result),
        }


    # --------- EMA LP filt

    def _update_wall(self, which, detection):
        """
        detection : (score, d, theta, n) or None
        returns   : (d_filtered, theta_filtered) or None
        """
        state_attr  = f'_{which}_state'
        missed_attr = f'_{which}_missed'
        state  = getattr(self, state_attr)
        missed = getattr(self, missed_attr)

        if detection is None:
            # no detection this frame
            missed += 1
            setattr(self, missed_attr, missed)
            if missed > self.MAX_MISSED:

                setattr(self, state_attr, None)
                return None
            return state   # hold last known value

        # valid detection arrived
        setattr(self, missed_attr, 0)
        _, d_new, theta_new, _ = detection

        if state is None:
            # first detection — initialise
            new_state = (d_new, theta_new)
        else:
            d_prev, theta_prev = state
            # EMA
            d_ema     = self.EMA_ALPHA * d_new + (1 - self.EMA_ALPHA) * d_prev
            theta_ema = theta_prev + self.EMA_ALPHA * angle_diff(theta_new, theta_prev)
            new_state = (d_ema, theta_ema)

        setattr(self, state_attr, new_state)
        return new_state    

    # ── Publishing ─────────────────────────────────────────────────────────────

    def _publish_wall(self, filtered, publisher, is_right: bool):
        msg = WallPose()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        label = 'Right' if is_right else 'Front'
        if filtered is None:
            msg.valid = False
            publisher.publish(msg)
            # Publish a fresh WallStatus marking the wall invalid so the FSM stops
            # trusting a stale valid pose (right wall only, not during CORNER_LOGIC).
            if is_right and self.robot_state != 'CORNER_LOGIC':
                status = WallStatus()
                status.header.stamp    = self.get_clock().now().to_msg()
                status.header.frame_id = 'base_link'
                status.wall_valid      = False
                status.aligned         = False
                status.distance_error  = 0.0
                status.angle_error     = 0.0
                status.distance_error_corrected = 0.0
                self._status_pub.publish(status)
            self.get_logger().info(
                f'{label} wall — NOT DETECTED',
                throttle_duration_sec=5.0)
            return

        d, theta = filtered
        target_theta = RIGHT_WALL_THETA if is_right else FRONT_WALL_THETA
        angle_error  = angle_diff(theta, target_theta)

        msg.distance = float(d)
        msg.angle    = float(angle_error)
        msg.valid    = True
        publisher.publish(msg)

        if is_right:
            dist_error = d - self.target_dist
            dist_error_corrected = (
                dist_error
                + self.lidar_offset_x * math.sin(angle_error)
                + self.lidar_offset_y * (1.0 - math.cos(angle_error))
            )
            aligned    = (abs(dist_error) < self.dist_tol and
                        abs(angle_error) < self.angle_tol)

            if self.robot_state != 'CORNER_LOGIC':
                status = WallStatus()
                status.header.stamp    = self.get_clock().now().to_msg()
                status.header.frame_id = 'base_link'
                status.wall_valid      = True
                status.aligned         = bool(aligned)
                status.distance_error  = float(dist_error)
                status.distance_error_corrected = float(dist_error_corrected)
                status.angle_error     = float(angle_error)
                self._status_pub.publish(status)

            self.get_logger().info(
                f'{label} Lidar->Wall — dist={d:.3f}m target={self.target_dist:.3f}m '
                f'dist_err={dist_error:+.3f}m Center->Wall- dist={dist_error_corrected:+.3f}m '
                f'angle_err={math.degrees(angle_error):.1f}deg aligned={aligned}',
                throttle_duration_sec=1.0)    
        else:
            self.get_logger().info(
                f'{label} wall — dist={d:.3f}m '
                f'angle_err={math.degrees(angle_error):.1f}deg',
                throttle_duration_sec=5.0)
            

        # d, theta = filtered
        # target_theta = RIGHT_WALL_THETA if is_right else FRONT_WALL_THETA
        # angle_error  = angle_diff(theta, target_theta)

        # msg.distance = float(d)
        # msg.angle    = float(angle_error)
        # msg.valid    = True
        # publisher.publish(msg)

        # # Publish wall_status for right wall only, not during CORNER_LOGIC
        # if is_right and self.robot_state != 'CORNER_LOGIC':
        #     dist_error = d - self.target_dist
        #     aligned    = (abs(dist_error) < self.dist_tol and
        #                   abs(angle_error) < self.angle_tol)
        #     status = WallStatus()
        #     status.header.stamp    = self.get_clock().now().to_msg()
        #     status.header.frame_id = 'base_link'
        #     status.wall_valid      = True
        #     status.aligned         = bool(aligned)
        #     status.distance_error  = float(dist_error)
        #     status.angle_error     = float(angle_error)
        #     self._status_pub.publish(status)

        # if is_right:
        #     self.get_logger().info(
        #         f'{label} wall — dist={d:.3f}m target={self.target_dist:.3f}m '
        #         f'dist_err={dist_error:+.3f}m '
        #         f'angle_err={math.degrees(angle_error):.1f}deg '
        #         f'aligned={aligned}',
        #         throttle_duration_sec=1.0)
        # else:
        #     self.get_logger().info(
        #         f'{label} wall — dist={d:.3f}m '
        #         f'angle_err={math.degrees(angle_error):.1f}deg',
        #         throttle_duration_sec=5.0)

    def _publish_markers(self, right, front):
        ma  = MarkerArray()
        t   = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.header.frame_id = 'base_link'
        delete_all.header.stamp    = t
        delete_all.action          = Marker.DELETEALL
        ma.markers.append(delete_all)

        def wall_marker(candidate, color, marker_id):
            if candidate is None:
                return
            score, d, theta, n = candidate
            dir_vec = np.array([-n[1], n[0]])
            p0      = n * d          # foot of normal from origin
            p1      = p0 + dir_vec * 3.0
            p2      = p0 - dir_vec * 3.0

            # ── fitted line ──────────────────────────────────────────
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp    = t
            m.ns              = 'walls'
            m.id              = marker_id
            m.type            = Marker.LINE_STRIP
            m.scale.x         = 0.04
            m.color.r, m.color.g, m.color.b = color
            m.color.a         = 1.0
            m.points          = [
                Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
                Point(x=float(p2[0]), y=float(p2[1]), z=0.0),
            ]
            ma.markers.append(m)

            # ── normal arrow ─────────────────────────────────────────
            m2 = Marker()
            m2.header.frame_id = 'base_link'
            m2.header.stamp    = t
            m2.ns              = 'normals'
            m2.id              = marker_id + 100
            m2.type            = Marker.ARROW
            m2.scale.x         = 0.06
            m2.scale.y         = 0.12
            m2.scale.z         = 0.12
            m2.color.r         = 1.0
            m2.color.g         = 1.0
            m2.color.b         = 0.0
            m2.color.a         = 1.0
            m2.points          = [
                Point(x=float(p0[0]), y=float(p0[1]), z=0.0),
                Point(x=float(p0[0] + n[0]), y=float(p0[1] + n[1]), z=0.0),
            ]
            ma.markers.append(m2)

        wall_marker(right, (0.0, 1.0, 0.0), 1)   # green  — right wall
        wall_marker(front, (0.0, 0.7, 1.0), 2)   # cyan   — front wall

        self._marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = WallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
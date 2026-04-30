#!/usr/bin/env python3
import math
import time
import struct
from collections import deque
from typing import List, Optional, Tuple


import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from gap_explorer_interfaces.action import ProbeArm
from rclpy.qos import qos_profile_sensor_data

def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

def quat_from_yaw(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.array([1.0, 0.0], dtype=float)

class GapExplorer(Node):
    def __init__(self):
        super().__init__('gap_explorer')


        self.declare_parameter('startup_scan_sec', 3.0)
        self.declare_parameter('post_nav_pause_sec', 2.0)
        self.declare_parameter('standoff_distance', 0.95)
        self.declare_parameter('follow_side', 'auto')  
        self.declare_parameter('follow_speed', 0.19)
        self.declare_parameter('max_ang_speed', 0.9)
        self.declare_parameter('heading_kp', 2.2)
        self.declare_parameter('cross_track_kp', 1.6)
        self.declare_parameter('lookahead_m', 0.40)
        self.declare_parameter('endpoint_reach_margin_m', 0.10)
        self.declare_parameter('probe_pause_sec', 1.5)

        self.declare_parameter('max_candidate_range_m', 4.0)
        self.declare_parameter('max_range_jump_m', 0.10)
        self.declare_parameter('min_segment_points', 15)
        self.declare_parameter('min_segment_length_m', 0.60)
        self.declare_parameter('max_fit_rmse_m', 0.05)

        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('occupancy_threshold', 99.0)
        self.declare_parameter('path_check_step_m', 0.05)

        self.declare_parameter('completed_wall_match_dist_m', 0.1)
        self.declare_parameter('completed_wall_match_angle_deg', 25.0)
        self.declare_parameter('completed_wall_match_length_m', 1.00)

        self.declare_parameter('explore_goal_min_m', 0.5)
        self.declare_parameter('explore_goal_max_m', 3.2)
        self.declare_parameter('explore_goal_clearance_m', 0.1)
        self.declare_parameter('explore_goal_heading_weight', 0.35)
        self.declare_parameter('explore_goal_distance_weight', -0.25)
        self.declare_parameter('follow_clearance_radius_m', 0.125)
        self.declare_parameter('loop_recovery_events_before_explore', 2)
        self.declare_parameter('loop_recovery_cooldown_sec', 8.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.15)

        self.declare_parameter('explore_probe_radius_m', 0.25)
        self.declare_parameter('explore_open_probe_points', 8)
        self.declare_parameter('explore_turn_weight', 1.2)
        self.declare_parameter('explore_open_weight', 0.6)

        self.declare_parameter('explore_sample_min_m', 0.5)
        self.declare_parameter('explore_sample_max_m', 2.0)
        self.declare_parameter('explore_cost_weight', 2.0)
        self.declare_parameter('explore_dist_weight', 0.45)
        self.declare_parameter('explore_dir_weight', 0.6)
        self.declare_parameter('explore_max_cost', 5)
        

        ##Action Stuff##
        self.probe_client = ActionClient(self, ProbeArm, 'probe_arm')
        self.probe_goal_handle = None
        self.probe_result_future = None
        self.probe_in_progress = False
        self.last_probe_detected = None
        self._probed_wall = None
        self._approach_strike_count = 0
        self._approach_strike_wall = None
        self._approach_strike_limit = 3


        self.declare_parameter('nav_danger_radius_m', 0.3)
        self.declare_parameter('nav_danger_cost', 98)
        self.declare_parameter('nav_safety_check_period_sec', 0.5)
        self.nav_danger_radius_m = float(self.get_parameter('nav_danger_radius_m').value)
        self.nav_danger_cost = int(self.get_parameter('nav_danger_cost').value)
        self.nav_safety_check_period_sec = float(self.get_parameter('nav_safety_check_period_sec').value)
        self.last_nav_safety_check = 0.0
        self.last_nav_safety_check = 0.0
        self.recently_aborted_walls: List[dict] = []
        self.recently_aborted_ttl_sec = 30.0
        self.explore_max_cost = int(self.get_parameter('explore_max_cost').value)
        self.explore_sample_min_m = float(self.get_parameter('explore_sample_min_m').value)
        self.explore_sample_max_m = float(self.get_parameter('explore_sample_max_m').value)
        self.explore_cost_weight = float(self.get_parameter('explore_cost_weight').value)
        self.explore_dist_weight = float(self.get_parameter('explore_dist_weight').value)
        self.explore_dir_weight = float(self.get_parameter('explore_dir_weight').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.follow_clearance_radius_m = float(self.get_parameter('follow_clearance_radius_m').value)
        self.explore_goal_clearance_m = float(self.get_parameter('explore_goal_clearance_m').value)
        self.nav_purpose = None
        self.startup_scan_sec = float(self.get_parameter('startup_scan_sec').value)
        self.post_nav_pause_sec = float(self.get_parameter('post_nav_pause_sec').value)
        self.standoff = float(self.get_parameter('standoff_distance').value)
        self.initial_follow_side = str(self.get_parameter('follow_side').value)
        self.follow_speed = float(self.get_parameter('follow_speed').value)
        self.max_ang_speed = float(self.get_parameter('max_ang_speed').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.cross_track_kp = float(self.get_parameter('cross_track_kp').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.endpoint_margin = float(self.get_parameter('endpoint_reach_margin_m').value)

        self.max_candidate_range_m = float(self.get_parameter('max_candidate_range_m').value)
        self.max_range_jump_m = float(self.get_parameter('max_range_jump_m').value)
        self.min_segment_points = int(self.get_parameter('min_segment_points').value)
        self.min_segment_length_m = float(self.get_parameter('min_segment_length_m').value)
        self.max_fit_rmse_m = float(self.get_parameter('max_fit_rmse_m').value)

        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.occ_th = int(self.get_parameter('occupancy_threshold').value)
        self.path_check_step_m = float(self.get_parameter('path_check_step_m').value)

        self.completed_wall_match_dist_m = float(self.get_parameter('completed_wall_match_dist_m').value)
        self.completed_wall_match_angle = math.radians(float(self.get_parameter('completed_wall_match_angle_deg').value))
        self.completed_wall_match_length_m = float(self.get_parameter('completed_wall_match_length_m').value)

        self.declare_parameter('scan_topic', '/scan')
        self.scan_topic = self.get_parameter('scan_topic').value

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/gap_debug_markers', 10)
        self.mirror_cloud_pub = self.create_publisher(PointCloud2, '/detected_mirrors', 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.timer = self.create_timer(0.1, self.step)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan: Optional[LaserScan] = None
        self.scan_buffer = deque(maxlen=50)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.costmap: Optional[OccupancyGrid] = None
        self.cost_np: Optional[np.ndarray] = None
        self.state = 'COLLECT'
        self.state_deadline = time.time() + self.startup_scan_sec
        self.nav_status = None
        self.nav_goal_handle = None
        self.last_explore_yaw: Optional[float] = None
        self.active_follow_side: Optional[str] = None
        self.locked_wall: Optional[dict] = None
        self.wall_travel_t: Optional[np.ndarray] = None
        self.wall_away_n: Optional[np.ndarray] = None
        self.wall_start_point: Optional[Tuple[float, float]] = None
        self.wall_goal_point: Optional[Tuple[float, float]] = None
        self.wall_lateral_bias = 0.0
        self.wall_last_local_shift = 0.0

        self.last_segments: List[dict] = []

        self.completed_walls: List[dict] = []
        self.detected_mirrors: List[dict] = []

    # ---------------- callbacks ----------------

    def scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.scan_buffer.append(msg)

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg
        self.cost_np = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

    # ---------------- low-level helpers ----------------

    def add_recently_aborted(self, wall: Optional[dict]):
        if wall is None:
            return
        sig = self.canonical_wall_signature(wall)
        sig['expires_at'] = time.time() + self.recently_aborted_ttl_sec
        self.recently_aborted_walls.append(sig)

    def prune_recently_aborted(self):
        now = time.time()
        self.recently_aborted_walls = [
            w for w in self.recently_aborted_walls if w['expires_at'] > now
        ]

    def is_recently_aborted(self, candidate_sig: dict) -> bool:
        self.prune_recently_aborted()
        for w in self.recently_aborted_walls:
            d = float(np.linalg.norm(candidate_sig['center'] - w['center']))
            ang_diff = abs(wrap(candidate_sig['angle'] - w['angle']))
            ang_diff = min(ang_diff, math.pi - ang_diff)
            if (d < self.completed_wall_match_dist_m and
                    ang_diff < self.completed_wall_match_angle):
                return True
        return False
    
    def cancel_nav_goal(self, reason: str = ''):
        """Cancel the currently active Nav2 goal, if any."""
        if self.nav_goal_handle is not None:
            self.get_logger().info(f'Cancelling nav goal: {reason}')
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'Cancel request failed: {e}')
            self.nav_goal_handle = None
        self.nav_status = 'failed'

    def nav_path_unsafe(self) -> Tuple[bool, str]:
        """
        Check if the robot is currently in a dangerous spot during NAV.
        Returns (unsafe, reason). 
        
        Two checks:
        1. Robot's immediate surroundings have lethal cost (it's close to or
        already inside an obstacle).
        2. The straight-line path from robot to goal crosses lethal cost.
        Conservative — Nav2 might be planning around it, but if the goal
        direction is blocked, the wall side was likely a bad pick.
        """
        if self.costmap is None or self.locked_wall is None:
            return False, ''
        
        info = self.costmap.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = info.width
        h = info.height
        data = self.costmap.data
        
        def cost_at(wx: float, wy: float) -> int:
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            if gx < 0 or gx >= w or gy < 0 or gy >= h:
                return -1  # off-map: treat as unknown
            return int(data[gy * w + gx])
        
        # Check 1: robot's immediate surroundings
        radius_cells = max(1, int(self.nav_danger_radius_m / res))
        bad = 0
        sampled = 0
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                wx = self.x + dx * res
                wy = self.y + dy * res
                c = cost_at(wx, wy)
                if c >= 0:
                    sampled += 1
                    if c >= self.nav_danger_cost:
                        bad += 1
        if sampled > 0 and bad / sampled > 0.30:  # >15% of nearby cells lethal
            return True, f'{bad}/{sampled} cells near robot have lethal cost'
        
        # # Check 2: straight-line path from robot toward the wall start point
        # if self.wall_start_point is None:
        #     return False, ''
        
        # gx = float(self.wall_start_point[0])
        # gy = float(self.wall_start_point[1])
        # dx = gx - self.x
        # dy = gy - self.y
        # dist = math.hypot(dx, dy)
        # if dist < 0.1:
        #     return False, ''
        
        # steps = max(1, int(dist / self.path_check_step_m))
        # blocked = 0
        # for i in range(1, steps + 1):
        #     f = i / steps
        #     sx = self.x + f * dx
        #     sy = self.y + f * dy
        #     c = cost_at(sx, sy)
        #     if c >= self.nav_danger_cost:
        #         blocked += 1
        # if blocked >= 2:  # at least 2 path cells with lethal cost
        #     return True, f'{blocked} cells on path to wall have lethal cost'
        
        return False, ''
    def probe_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Probe feedback: {fb.state}')

    def probe_goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.finish_probe(False, False, 'Probe goal rejected')
            return

        self.probe_goal_handle = goal_handle
        self.probe_result_future = goal_handle.get_result_async()
        self.probe_result_future.add_done_callback(self.probe_result_cb)

    def probe_result_cb(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            self.finish_probe(
                bool(result.success),
                bool(result.object_detected),
                str(result.message)
            )
        except Exception as e:
            self.finish_probe(False, False, f'Probe exception: {e}')

    def finish_probe(self, success: bool, object_detected: bool, message: str):
        self.probe_in_progress = False
        self.probe_goal_handle = None
        self.probe_result_future = None
        self.last_probe_detected = object_detected

        self.get_logger().info(
            f'Probe finished: success={success}, object_detected={object_detected}, msg={message}'
        )

        if object_detected and self._probed_wall is not None:
            self.remember_mirror(self._probed_wall)

        self._probed_wall = None

        # Clear wall-follow state and continue only after probe completes
        self.clear_locked_wall()
        self.scan_buffer.clear()
        self.scan = None
        self.last_segments = []
        self.state_deadline = time.time() + self.startup_scan_sec
        self.state = 'COLLECT'

    def shorten_plan_until_safe(self, plan: dict) -> Optional[dict]:
        start = np.array(plan['start_point'], dtype=float)
        goal = np.array(plan['goal_point'], dtype=float)
        t = np.array(plan['travel_t'], dtype=float)

        L = float(np.linalg.norm(goal - start))
        if L < 1e-6:
            return None

        # Start point itself must be safe enough to nav to
        if not self.free_with_clearance(float(start[0]), float(start[1]), self.explore_goal_clearance_m):
            return None

        # If full standoff path is already safe, keep it
        if not self.line_blocked(start, goal, self.follow_clearance_radius_m):
            return plan

        # Otherwise trim back from the far endpoint along the same line
        min_len = max(self.endpoint_margin + 0.05, 0.20)
        trim = .1

        while L >= min_len:
            new_goal = start + L * t

            if self.free_with_clearance(float(new_goal[0]), float(new_goal[1]), self.follow_clearance_radius_m):
                if not self.line_blocked(start, new_goal, self.follow_clearance_radius_m):
                    new_plan = dict(plan)
                    new_plan['goal_point'] = (float(new_goal[0]), float(new_goal[1]))
                    return new_plan

            L -= trim

        return None


    def cost_at(self, x: float, y: float) -> int:
        if self.costmap is None or self.cost_np is None:
            return 100
        info = self.costmap.info
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return 100
        return int(self.cost_np[iy, ix])

    def candidate_near_completed_wall(self, wall: dict) -> bool:
        sig = self.canonical_wall_signature(wall)

        for done in self.completed_walls:
            da = abs(sig['angle'] - done['angle'])
            da = min(da, abs(da - math.pi))

            # only compare walls that are roughly parallel
            if da > self.completed_wall_match_angle:
                continue

            # reject if either endpoint is close to the completed wall segment
            d0 = self.point_to_segment_distance(sig['p0'], done['p0'], done['p1'])
            d1 = self.point_to_segment_distance(sig['p1'], done['p0'], done['p1'])

            # also reject if the completed-wall endpoints are close to the candidate segment
            d2 = self.point_to_segment_distance(done['p0'], sig['p0'], sig['p1'])
            d3 = self.point_to_segment_distance(done['p1'], sig['p0'], sig['p1'])

            min_sep = min(d0, d1, d2, d3)

            if min_sep < self.completed_wall_match_dist_m:
                return True

        return False

    def point_to_segment_distance(self, p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        ab = b - a
        L2 = float(np.dot(ab, ab))
        if L2 < 1e-9:
            return float(np.linalg.norm(p - a))
        t = float(np.dot(p - a, ab) / L2)
        t = max(0.0, min(1.0, t))
        proj = a + t * ab
        return float(np.linalg.norm(p - proj))

    def update_pose_from_tf(self) -> bool:
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'No TF {self.global_frame}->{self.robot_base_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return False

        t = tfm.transform.translation
        r = tfm.transform.rotation

        self.x = float(t.x)
        self.y = float(t.y)
        self.yaw = yaw_from_quat(r)
        return True

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(max(-self.max_ang_speed, min(self.max_ang_speed, wz)))
        self.cmd_pub.publish(t)

    def robot_to_world(self, xr: float, yr: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * xr - s * yr, self.y + s * xr + c * yr

    def occupied(self, x: float, y: float) -> bool:
        if self.costmap is None or self.cost_np is None:
            return False
        info = self.costmap.info
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return True
        return int(self.cost_np[iy, ix]) >= self.occ_th

    def line_blocked(self, p0: np.ndarray, p1: np.ndarray, radius: Optional[float] = None) -> bool:
        if radius is None:
            radius = self.follow_clearance_radius_m

        d = p1 - p0
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return not self.free_with_clearance(float(p0[0]), float(p0[1]), radius)

        t = d / L
        n_samples = max(2, int(L / self.path_check_step_m) + 1)

        for s in np.linspace(0.0, L, n_samples):
            q = p0 + s * t
            if not self.free_with_clearance(float(q[0]), float(q[1]), radius):
                return True
        return False

    def free_with_clearance(self, x: float, y: float, radius: float) -> bool:
        if self.costmap is None:
            return True

        info = self.costmap.info
        res = max(info.resolution, 1e-3)
        steps = max(1, int(radius / res))

        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                dx = ix * res
                dy = iy * res
                if dx * dx + dy * dy > radius * radius:
                    continue
                px = x + dx
                py = y + dy
                if self.occupied(px, py):
                    return False
        return True

    def choose_explore_goal(self) -> Optional[Tuple[float, float, float]]:
        if self.costmap is None or self.cost_np is None:
            return None

        robot = np.array([self.x, self.y], dtype=float)

        # Short local hops only
        radii = np.linspace(self.explore_sample_min_m, self.explore_sample_max_m, 7)
        # Mostly forward, but allow some spread
        rel_angles = np.deg2rad(np.linspace(-90.0, 90.0, 17))

        candidates = []

        for r in radii:
            for a in rel_angles:
                gyaw = wrap(self.yaw + float(a))
                gx = self.x + float(r) * math.cos(gyaw)
                gy = self.y + float(r) * math.sin(gyaw)
                goal = np.array([gx, gy], dtype=float)

                # Goal must be safe enough
                if not self.free_with_clearance(gx, gy, self.explore_goal_clearance_m):
                    continue

                # Path must be safe enough
                if self.line_blocked(robot, goal, self.explore_goal_clearance_m):
                    continue

                raw_cost = self.cost_at(gx, gy)

                # Only accept truly free / near-zero-cost cells for exploration
                if raw_cost < 0 or raw_cost > self.explore_max_cost:
                    continue

                # Normalize cost into [0,1] roughly
                cost_norm = max(0.0, min(1.0, raw_cost / max(1.0, self.occ_th)))

                # Mild preference for continuing prior explore heading
                dir_penalty = 0.0
                if self.last_explore_yaw is not None:
                    dir_penalty = abs(wrap(gyaw - self.last_explore_yaw))

                # Mild preference for moving a bit farther, but still local
                dist_norm = (float(r) - self.explore_sample_min_m) / max(
                    1e-6, self.explore_sample_max_m - self.explore_sample_min_m
                )

                # Lower score wins:
                # strongly prefer low cost,
                # mildly prefer farther,
                # mildly prefer same direction as last explore
                score = (
                    self.explore_cost_weight * cost_norm
                    - self.explore_dist_weight * dist_norm
                    + self.explore_dir_weight * dir_penalty
                )

                candidates.append((score, gx, gy, gyaw, raw_cost, float(r), dir_penalty))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0])
        _, gx, gy, gyaw, raw_cost, dist, dir_penalty = candidates[0]

        self.get_logger().info(
            f'Explore goal: ({gx:.2f}, {gy:.2f}), yaw={gyaw:.2f}, '
            f'cost={raw_cost}, dist={dist:.2f}, dir_penalty={dir_penalty:.2f}'
        )

        return gx, gy, gyaw
    # ---------------- completed-wall memory ----------------

    def canonical_wall_signature(self, wall: dict) -> dict:
        p0 = np.array(wall['p0_w'], dtype=float)
        p1 = np.array(wall['p1_w'], dtype=float)

        # canonical ordering so same wall compares the same regardless of direction
        if tuple(p1.tolist()) < tuple(p0.tolist()):
            p0, p1 = p1, p0

        c = 0.5 * (p0 + p1)
        t = unit(p1 - p0)

        # angle modulo pi so reversed direction is treated as same wall
        ang = math.atan2(t[1], t[0])
        if ang < 0.0:
            ang += math.pi
        if ang >= math.pi:
            ang -= math.pi

        length = float(np.linalg.norm(p1 - p0))

        return {
            'p0': p0,
            'p1': p1,
            'center': c,
            'angle': ang,
            'length': length,
        }

    def completed_wall_matches(self, wall: dict) -> bool:
        sig = self.canonical_wall_signature(wall)

        for done in self.completed_walls:
            d_center = float(np.linalg.norm(sig['center'] - done['center']))

            da = abs(sig['angle'] - done['angle'])
            da = min(da, abs(da - math.pi))

            dl = abs(sig['length'] - done['length'])

            # endpoint consistency check to catch same wall from opposite side / opposite direction
            e00 = float(np.linalg.norm(sig['p0'] - done['p0']))
            e11 = float(np.linalg.norm(sig['p1'] - done['p1']))
            e01 = float(np.linalg.norm(sig['p0'] - done['p1']))
            e10 = float(np.linalg.norm(sig['p1'] - done['p0']))
            endpoint_err = min(e00 + e11, e01 + e10)

            if (
                d_center < self.completed_wall_match_dist_m and
                da < self.completed_wall_match_angle and
                dl < self.completed_wall_match_length_m
            ):
                return True

            # alternate match path: same physical segment by endpoints even if center/length drift a bit
            if endpoint_err < 2.0 * self.completed_wall_match_dist_m and da < self.completed_wall_match_angle:
                return True

        return False

    def remember_completed_wall(self, wall: dict):
        sig = self.canonical_wall_signature(wall)

        for done in self.completed_walls:
            d_center = float(np.linalg.norm(sig['center'] - done['center']))
            da = abs(sig['angle'] - done['angle'])
            da = min(da, abs(da - math.pi))
            dl = abs(sig['length'] - done['length'])

            e00 = float(np.linalg.norm(sig['p0'] - done['p0']))
            e11 = float(np.linalg.norm(sig['p1'] - done['p1']))
            e01 = float(np.linalg.norm(sig['p0'] - done['p1']))
            e10 = float(np.linalg.norm(sig['p1'] - done['p0']))
            endpoint_err = min(e00 + e11, e01 + e10)

            if (
                (d_center < self.completed_wall_match_dist_m and
                 da < self.completed_wall_match_angle and
                 dl < self.completed_wall_match_length_m)
                or
                (endpoint_err < 2.0 * self.completed_wall_match_dist_m and
                 da < self.completed_wall_match_angle)
            ):
                return

        self.completed_walls.append(sig)
        self.get_logger().info(
            f"Saved completed wall: center=({sig['center'][0]:.2f}, {sig['center'][1]:.2f}) "
            f"angle={math.degrees(sig['angle']):.1f} len={sig['length']:.2f}"
        )
        
    def mirror_matches(self, wall: dict) -> bool:
        """True if this wall signature is already in detected_mirrors."""
        sig = self.canonical_wall_signature(wall)
        for done in self.detected_mirrors:
            d_center = float(np.linalg.norm(sig['center'] - done['center']))
            da = abs(sig['angle'] - done['angle'])
            da = min(da, abs(da - math.pi))
            e00 = float(np.linalg.norm(sig['p0'] - done['p0']))
            e11 = float(np.linalg.norm(sig['p1'] - done['p1']))
            e01 = float(np.linalg.norm(sig['p0'] - done['p1']))
            e10 = float(np.linalg.norm(sig['p1'] - done['p0']))
            endpoint_err = min(e00 + e11, e01 + e10)
            if d_center < self.completed_wall_match_dist_m and da < self.completed_wall_match_angle:
                return True
            if endpoint_err < 2.0 * self.completed_wall_match_dist_m and da < self.completed_wall_match_angle:
                return True
        return False

    def remember_mirror(self, wall: dict):
        """Store a wall as a detected mirror (same format as completed_walls)."""
        if self.mirror_matches(wall):
            return
        sig = self.canonical_wall_signature(wall)
        self.detected_mirrors.append(sig)
        self.get_logger().info(
            f"Mirror detected: center=({sig['center'][0]:.2f}, {sig['center'][1]:.2f}) "
            f"angle={math.degrees(sig['angle']):.1f} len={sig['length']:.2f}"
        )
        self._publish_mirror_cloud()

    def _publish_mirror_cloud(self):
        """Republish all detected mirror centers as a PointCloud2 for Nav2 obstacle layer."""
        points = []
        for m in self.detected_mirrors:
            c = m['center']
            # Publish several points along the mirror segment so the costmap inflates a line, not just a dot
            t = np.array([math.cos(m['angle']), math.sin(m['angle'])], dtype=float)
            half = 0.5 * m['length']
            steps = max(3, int(m['length'] / 0.05))
            for i in range(steps + 1):
                s = -half + (2.0 * half * i / steps)
                p = c + s * t
                points.append((float(p[0]), float(p[1]), 0.1))

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = bytearray()
        for (x, y, z) in points:
            msg.data += struct.pack('fff', x, y, z)

        self.mirror_cloud_pub.publish(msg)
    # ---------------- wall extraction ----------------

    def scan_points_robot(self, scan: LaserScan):
        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < 0.03 or r > self.max_candidate_range_m:
                continue
            a = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a), r, i))
        return pts

    def contiguous_segments(self, pts):
        """
        Group points by spatial proximity instead of scan-index proximity.
        Robust to sparse data with dropouts.
        """
        if not pts:
            return []
        
        # Sort by angle so spatially adjacent points are sequential
        pts = sorted(pts, key=lambda p: math.atan2(p[1], p[0]))
        
        out, cur = [], [pts[0]]
        max_gap = 0.30  # meters between consecutive points in the same segment
        
        for p in pts[1:]:
            prev = cur[-1]
            # Spatial distance between successive points
            d = math.hypot(p[0] - prev[0], p[1] - prev[1])
            # Range jump kept as a sanity guard for absurd outliers
            rj = abs(p[2] - prev[2])
            if d <= max_gap and rj <= self.max_range_jump_m * 3.0:
                cur.append(p)
            else:
                if len(cur) >= self.min_segment_points:
                    out.append(cur)
                cur = [p]
        if len(cur) >= self.min_segment_points:
            out.append(cur)
        return out

    def fit_segment(self, seg) -> Optional[dict]:
        if len(seg) < self.min_segment_points:
            return None

        P = np.array([[p[0], p[1]] for p in seg], dtype=float)
        c = P.mean(axis=0)
        X = P - c
        vals, vecs = np.linalg.eigh((X.T @ X) / max(len(P) - 1, 1))
        t = unit(vecs[:, np.argmax(vals)])

        proj = X @ t
        rmse = float(np.sqrt(np.mean(np.sum((X - np.outer(proj, t)) ** 2, axis=1))))
        if rmse > self.max_fit_rmse_m:
            return None

        s0, s1 = float(np.min(proj)), float(np.max(proj))
        length = s1 - s0
        if length < self.min_segment_length_m:
            return None

        p0_r = c + s0 * t
        p1_r = c + s1 * t

        return {
            'p0_r': p0_r,
            'p1_r': p1_r,
            'c_r': c,
            't_r': t,
            'length': length,
            'rmse': rmse,
            'score': 3.0 * length - 0.7 * abs(float(np.linalg.norm(c)) - self.standoff)
        }

    def segment_to_world(self, s: dict) -> dict:
        p0w = np.array(self.robot_to_world(float(s['p0_r'][0]), float(s['p0_r'][1])))
        p1w = np.array(self.robot_to_world(float(s['p1_r'][0]), float(s['p1_r'][1])))
        cw = np.array(self.robot_to_world(float(s['c_r'][0]), float(s['c_r'][1])))
        ang = wrap(math.atan2(float(s['t_r'][1]), float(s['t_r'][0])) + self.yaw)

        return {
            **s,
            'p0_w': p0w,
            'p1_w': p1w,
            'c_w': cw,
            't_w': np.array([math.cos(ang), math.sin(ang)]),
            'angle_w': ang
        }

    def select_best_wall(self) -> Optional[dict]:
        all_segments = []
        for sc in list(self.scan_buffer)[-min(20, len(self.scan_buffer)):]:
            for raw in self.contiguous_segments(self.scan_points_robot(sc)):
                fit = self.fit_segment(raw)
                if fit is not None:
                    all_segments.append(self.segment_to_world(fit))

        all_segments = [
                    s for s in all_segments
                    if not self.completed_wall_matches(s)
                    and not self.candidate_near_completed_wall(s)
                    and not self.mirror_matches(s)
                    and not self.is_recently_aborted(self.canonical_wall_signature(s))
                ]
        self.last_segments = all_segments

        if not all_segments:
            return None

        all_segments.sort(key=lambda s: s['score'], reverse=True)
        return all_segments[0]

    # ---------------- wall plan logic ----------------

    def offset_plans_for_wall(self, wall: dict, follow_side: str) -> List[dict]:
        plans = []
        for a, b in ((wall['p0_w'], wall['p1_w']), (wall['p1_w'], wall['p0_w'])):
            t = unit(b - a)
            away = np.array([-t[1], t[0]]) if follow_side == 'right' else np.array([t[1], -t[0]])
            start = a + self.standoff * away
            goal = b + self.standoff * away

            plan = {
                'segment': wall,
                'start_point': (float(start[0]), float(start[1])),
                'goal_point': (float(goal[0]), float(goal[1])),
                'heading': math.atan2(t[1], t[0]),
                'travel_t': t.copy(),
                'away_n': away.copy(),
                'robot_dist': float(np.linalg.norm(start - np.array([self.x, self.y]))),
                'follow_side': follow_side
            }

            safe_plan = self.shorten_plan_until_safe(plan)
            if safe_plan is not None:
                plans.append(safe_plan)

        return plans


    def choose_initial_plan(self, wall: dict) -> dict:
        if self.initial_follow_side in ('right', 'left'):
            return self.offset_plans_for_wall(wall, self.initial_follow_side)[0]

        candidates = []
        for side in ('right', 'left'):
            candidates.extend(self.offset_plans_for_wall(wall, side))

        best = None
        best_score = 1e9
        robot = np.array([self.x, self.y], dtype=float)

        for c in candidates:
            start = np.array(c['start_point'], dtype=float)
            goal = np.array(c['goal_point'], dtype=float)
            robot_dist = float(np.linalg.norm(start - robot))
            plan_len = float(np.linalg.norm(goal - start))

            score = robot_dist - 0.5 * plan_len
            if score < best_score:
                best_score = score
                best = c

        return best
    
    def lock_plan(self, plan: dict):
        self.locked_wall = dict(plan['segment'])
        self.wall_travel_t = plan['travel_t'].copy()
        self.wall_away_n = plan['away_n'].copy()
        self.wall_start_point = plan['start_point']
        self.wall_goal_point = plan['goal_point']
        self.active_follow_side = plan['follow_side']
        self.wall_lateral_bias = 0.0
        self.wall_last_local_shift = 0.0

    def clear_locked_wall(self):
        self.locked_wall = None
        self.wall_travel_t = None
        self.wall_away_n = None
        self.wall_start_point = None
        self.wall_goal_point = None
        self.active_follow_side = None
        self.wall_lateral_bias = 0.0
        self.wall_last_local_shift = 0.0

    def refresh_locked_wall(self):
        if self.scan is None or self.locked_wall is None or self.wall_travel_t is None or self.wall_away_n is None:
            return

        p0 = np.array(self.locked_wall['p0_w'], dtype=float)
        p1 = np.array(self.locked_wall['p1_w'], dtype=float)

        # Keep endpoint ordering consistent with the current travel direction.
        if float(np.dot(p1 - p0, self.wall_travel_t)) >= 0.0:
            start = p0.copy()
            end = p1.copy()
        else:
            start = p1.copy()
            end = p0.copy()

        t = unit(self.wall_travel_t)
        n = np.array([-t[1], t[0]], dtype=float)

        current_len = float(np.dot(end - start, t))
        if current_len < 0.0:
            current_len = 0.0

        robot = np.array([self.x, self.y], dtype=float)
        s_robot = float(np.dot(robot - start, t))
        s_robot = max(0.0, min(current_len, s_robot))

        # Local lateral update near the robot only.
        local_back_m = 0.25
        local_forward_m = 0.45
        local_lateral_tol = 0.12
        local_min_points = 5
        local_max_index_jump = 2
        max_local_shift_per_cycle = 0.02
        local_shift_alpha = 0.15
        max_total_lateral_bias = 0.10

        # Forward growth gate. This only affects length, not lateral position.
        growth_back_m = 0.60
        growth_lateral_tol = 0.16
        min_forward_extension = 0.10
        max_growth_gap_m = 0.22
        min_forward_points = 4

        proj_pts = []
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < 0.03 or r > self.max_candidate_range_m:
                continue
            a = self.scan.angle_min + i * self.scan.angle_increment
            xr = r * math.cos(a)
            yr = r * math.sin(a)
            xw, yw = self.robot_to_world(xr, yr)
            p = np.array([xw, yw], dtype=float)

            rel = p - start
            s = float(np.dot(rel, t))
            e = float(np.dot(rel, n))
            proj_pts.append((p, s, e, i))

        if not proj_pts:
            return

        # ---------------- local lateral correction ----------------
        local_pts = [
            (p, s, e, i)
            for (p, s, e, i) in proj_pts
            if (s_robot - local_back_m) <= s <= (s_robot + local_forward_m)
            and abs(e) <= local_lateral_tol
        ]

        best_cluster = None
        if local_pts:
            local_pts.sort(key=lambda x: x[3])
            clusters = []
            cur = [local_pts[0]]
            for item in local_pts[1:]:
                if item[3] <= cur[-1][3] + local_max_index_jump:
                    cur.append(item)
                else:
                    clusters.append(cur)
                    cur = [item]
            if cur:
                clusters.append(cur)

            best_key = None
            for c in clusters:
                e_med_abs = float(np.median([abs(e) for (_, _, e, _) in c]))
                key = (-len(c), e_med_abs)
                if best_key is None or key < best_key:
                    best_key = key
                    best_cluster = c

        if best_cluster is not None and len(best_cluster) >= local_min_points:
            e_local = float(np.median([e for (_, _, e, _) in best_cluster]))
            e_local = max(-max_local_shift_per_cycle, min(max_local_shift_per_cycle, e_local))
            self.wall_lateral_bias = (
                (1.0 - local_shift_alpha) * self.wall_lateral_bias
                + local_shift_alpha * e_local
            )
            self.wall_lateral_bias = max(
                -max_total_lateral_bias,
                min(max_total_lateral_bias, self.wall_lateral_bias)
            )
            self.wall_last_local_shift = e_local
        else:
            self.wall_last_local_shift = 0.0

        shift_vec = self.wall_lateral_bias * n
        shifted_start = start + shift_vec
        shifted_end = end + shift_vec

        # ---------------- forward growth only ----------------
        growth_pts = []
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < 0.03 or r > self.max_candidate_range_m:
                continue
            a = self.scan.angle_min + i * self.scan.angle_increment
            xr = r * math.cos(a)
            yr = r * math.sin(a)
            xw, yw = self.robot_to_world(xr, yr)
            p = np.array([xw, yw], dtype=float)

            rel = p - shifted_start
            s = float(np.dot(rel, t))
            e = float(np.dot(rel, n))
            if s >= current_len - growth_back_m and abs(e) <= growth_lateral_tol:
                growth_pts.append((p, s, e, i))

        forward_pts = [
            (p, s, e, i)
            for (p, s, e, i) in growth_pts
            if s > current_len + min_forward_extension
        ]
        forward_pts.sort(key=lambda x: x[1])

        accepted = []
        prev_s = current_len
        for item in forward_pts:
            _, s, _, _ = item
            if s - prev_s > max_growth_gap_m:
                break
            accepted.append(item)
            prev_s = s

        new_len = current_len
        if len(accepted) >= min_forward_points:
            new_len = float(max(s for (_, s, _, _) in accepted))

        new_end = shifted_start + new_len * t

        self.locked_wall = {
            **self.locked_wall,
            'p0_w': shifted_start,
            'p1_w': new_end,
            'c_w': 0.5 * (shifted_start + new_end),
            't_w': t,
            'angle_w': math.atan2(t[1], t[0]),
            'length': float(np.linalg.norm(new_end - shifted_start)),
        }

        self.wall_travel_t = t
        if self.active_follow_side == 'right':
            self.wall_away_n = np.array([-t[1], t[0]], dtype=float)
        else:
            self.wall_away_n = np.array([t[1], -t[0]], dtype=float)
        self.wall_start_point = tuple((shifted_start + self.standoff * self.wall_away_n).tolist())
        self.wall_goal_point = tuple((new_end + self.standoff * self.wall_away_n).tolist())

    # ---------------- nav2 ----------------

    def send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return False

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz, qw = quat_from_yaw(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        self.nav_status = None
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, fut):
            self.nav_goal_handle = fut.result()
            self.get_logger().info(f'Nav goal accepted: {self.nav_goal_handle is not None and self.nav_goal_handle.accepted}')
            if self.nav_goal_handle is None or not self.nav_goal_handle.accepted:
                self.nav_status = 'failed'
                return
            rf = self.nav_goal_handle.get_result_async()
            rf.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, fut):
            result = fut.result()
            self.get_logger().info(f'Nav result callback fired: status={result.status if result else "None"}')
            self.nav_status = 'succeeded' if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED else 'failed'
            self.nav_goal_handle = None

    # ---------------- probe ----------------

    def start_probe(self):
        # Stop immediately — before any network calls
        self.stop()
        self.probe_in_progress = True
        self.state = 'PROBE'

        if not self.probe_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Probe action server not available')
            self.finish_probe(False, False, 'Probe server unavailable')
            return

        self._probed_wall = dict(self.locked_wall) if self.locked_wall is not None else None

        goal = ProbeArm.Goal()

        send_future = self.probe_client.send_goal_async(
            goal,
            feedback_callback=self.probe_feedback_cb
        )
        send_future.add_done_callback(self.probe_goal_response_cb)

    # ---------------- follow ----------------

    def lidar_follow_step(self):
        if self.locked_wall is None or self.wall_travel_t is None or self.wall_start_point is None or self.wall_goal_point is None:
            self.state = 'COLLECT'
            self.state_deadline = time.time() + self.startup_scan_sec
            return

        self.refresh_locked_wall()

        start = np.array(self.wall_start_point, dtype=float)
        goal = np.array(self.wall_goal_point, dtype=float)
        robot = np.array([self.x, self.y], dtype=float)

        path = goal - start
        L = float(np.linalg.norm(path))
        if L < 1e-6:
            self.stop()
            return

        t = path / L
        n = np.array([-t[1], t[0]])

        endpoint_dist = float(np.linalg.norm(robot - goal))
        if endpoint_dist <= self.endpoint_margin:
            if self.locked_wall is not None:
                self.remember_completed_wall(self.locked_wall)
            self.get_logger().info('Reached line endpoint, probing')
            self.start_probe()
            return

        s = max(0.0, min(L, float(np.dot(robot - start, t))))

        chase = start + min(L, s + self.lookahead_m) * t
        nearest = start + s * t

        safety_chase = start + min(L, s + max(self.lookahead_m, 0.70)) * t
        short_blocked = self.line_blocked(robot, safety_chase, self.follow_clearance_radius_m)

        if short_blocked:
            if self.locked_wall is not None:
                self.remember_completed_wall(self.locked_wall)

            self.stop()
            self.locked_wall = None
            self.wall_travel_t = None
            self.wall_away_n = None
            self.wall_start_point = None
            self.wall_goal_point = None
            self.active_follow_side = None
            self.wall_lateral_bias = 0.0
            self.wall_last_local_shift = 0.0

            self.scan_buffer.clear()
            self.scan = None
            self.last_segments = []

            self.state = 'COLLECT'
            self.state_deadline = time.time() + 2.0
            return

        cross = float(np.dot(robot - nearest, n))
        heading_err = wrap(math.atan2(chase[1] - self.y, chase[0] - self.x) - self.yaw)
        self.publish_cmd(self.follow_speed, self.heading_kp * heading_err - self.cross_track_kp * cross)

# ---------------- markers ----------------

    def publish_markers(self):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        mid = 0

        def add_line(p0, p1, ns, color, width=0.03):
            nonlocal mid
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.95

            a = Point()
            a.x, a.y, a.z = float(p0[0]), float(p0[1]), 0.05
            b = Point()
            b.x, b.y, b.z = float(p1[0]), float(p1[1]), 0.05
            m.points = [a, b]
            ma.markers.append(m)

        def add_sphere(p, ns, color, scale=0.10):
            nonlocal mid
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = 0.06
            m.pose.orientation.w = 1.0
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.95
            ma.markers.append(m)

        for s in self.last_segments:
            add_line(s['p0_w'], s['p1_w'], 'wall_segments', (0.5, 0.5, 0.5), 0.02)
            add_sphere(s['p0_w'], 'segment_endpoints', (0.5, 0.5, 0.5), 0.06)
            add_sphere(s['p1_w'], 'segment_endpoints', (0.5, 0.5, 0.5), 0.06)

        if self.locked_wall is not None:
            add_line(self.locked_wall['p0_w'], self.locked_wall['p1_w'], 'wall_match', (1.0, 1.0, 0.0), 0.05)

        if self.wall_start_point is not None and self.wall_goal_point is not None:
            start = np.array(self.wall_start_point, dtype=float)
            goal = np.array(self.wall_goal_point, dtype=float)
            add_line(start, goal, 'standoff_path', (1.0, 0.0, 1.0), 0.04)
            add_sphere(start, 'offset_endpoints', (1.0, 0.3, 1.0), 0.09)
            add_sphere(goal, 'offset_endpoints', (1.0, 1.0, 1.0), 0.11)

            robot = np.array([self.x, self.y], dtype=float)
            t = unit(goal - start)
            L = float(np.linalg.norm(goal - start))
            s = max(0.0, min(L, float(np.dot(robot - start, t))))
            chase = start + min(L, s + self.lookahead_m) * t
            add_sphere(chase, 'chase_point', (1.0, 0.0, 0.0), 0.12)

        for done in self.completed_walls:
            c = done['center']
            t = np.array([math.cos(done['angle']), math.sin(done['angle'])], dtype=float)
            half = 0.5 * done['length'] * t
            add_line(c - half, c + half, 'completed_walls', (0.2, 0.2, 0.2), 0.04)

        for m in self.detected_mirrors:
            c = m['center']
            t = np.array([math.cos(m['angle']), math.sin(m['angle'])], dtype=float)
            half = 0.5 * m['length'] * t
            add_line(c - half, c + half, 'detected_mirrors', (0.0, 0.8, 1.0), 0.06)
            add_sphere(c, 'mirror_centers', (0.0, 0.8, 1.0), 0.14)
        
        self.marker_pub.publish(ma)

    # ---------------- state machine ----------------

    def step(self):
        if not self.update_pose_from_tf():
            self.publish_markers()
            return

        if self.state == 'COLLECT':
            if time.time() >= self.state_deadline:
                
                best = self.select_best_wall()
                if best is None:
                    explore_goal = self.choose_explore_goal()
                    if explore_goal is None:
                        self.get_logger().info('No wall candidate and no safe explore goal; rescanning')
                        self.state_deadline = time.time() + self.startup_scan_sec
                    else:
                        gx, gy, gyaw = explore_goal
                        self.get_logger().info(
                            f'No wall candidate; exploring to ({gx:.2f}, {gy:.2f})'
                        )
                        ok = self.send_nav_goal(gx, gy, gyaw)
                        if ok:
                            self.last_explore_yaw = gyaw
                            self.nav_purpose = 'explore'
                            self.state = 'NAV'
                        else:
                            self.state_deadline = time.time() + self.startup_scan_sec
                else:
                    plan = self.choose_initial_plan(best)
                    if plan is None:
                        # Check if we keep failing on the same wall
                        best_sig = self.canonical_wall_signature(best)
                        if (self._approach_strike_wall is not None and
                                float(np.linalg.norm(
                                    best_sig['center'] - self._approach_strike_wall['center']
                                )) < self.completed_wall_match_dist_m):
                            self._approach_strike_count += 1
                        else:
                            self._approach_strike_count = 1
                            self._approach_strike_wall = best_sig

                        if self._approach_strike_count >= self._approach_strike_limit:
                            self.get_logger().warn(
                                f'Wall at ({best_sig["center"][0]:.2f}, {best_sig["center"][1]:.2f}) '
                                f'failed approach {self._approach_strike_count} times, skipping it'
                            )
                            self.remember_completed_wall(best)
                            self._approach_strike_count = 0
                            self._approach_strike_wall = None
                            # Try to explore somewhere new instead
                            explore_goal = self.choose_explore_goal()
                            if explore_goal is not None:
                                gx, gy, gyaw = explore_goal
                                ok = self.send_nav_goal(gx, gy, gyaw)
                                if ok:
                                    self.last_explore_yaw = gyaw
                                    self.nav_purpose = 'explore'
                                    self.state = 'NAV'
                                    return
                            self.state_deadline = time.time() + self.startup_scan_sec
                        else:
                            self.get_logger().warn(
                                f'No safe approach plan for selected wall '
                                f'(strike {self._approach_strike_count}/{self._approach_strike_limit}); rescanning'
                            )
                            self.state_deadline = time.time() + self.startup_scan_sec
                    else:
                        self._approach_strike_count = 0
                        self._approach_strike_wall = None
                        self.lock_plan(plan)
                        self.get_logger().info(
                            f"Selected a new wall. Following wall on {self.active_follow_side}. Sending Nav2 goal"
                        )
                        ok = self.send_nav_goal(
                            float(plan['start_point'][0]),
                            float(plan['start_point'][1]),
                            float(plan['heading'])
                        )
                        if ok:
                            self.nav_purpose = 'wall_start'
                            self.state = 'NAV'
                        else:
                            self.state = 'FOLLOW'


        elif self.state == 'NAV':
                # Safety bail-out (only for wall_start nav, not explore)
            if (self.nav_purpose == 'wall_start' and
                    self.nav_status is None and
                    time.time() - self.last_nav_safety_check >= self.nav_safety_check_period_sec):
                self.last_nav_safety_check = time.time()
                unsafe, reason = self.nav_path_unsafe()
                if unsafe:
                    self.cancel_nav_goal(f'safety: {reason}')
                    self.get_logger().warn(
                        f'Aborting wall approach — {reason}. Skipping wall for {self.recently_aborted_ttl_sec:.0f}s.'
                    )
                    if self.locked_wall is not None:
                        self.add_recently_aborted(self.locked_wall)
                    self.locked_wall = None
                    self.wall_start_point = None
                    self.wall_goal_point = None
                    self.nav_purpose = None
                    self.state = 'COLLECT'
                    self.state_deadline = time.time() + self.startup_scan_sec
                    return
            if self.nav_status == 'succeeded':
                self.nav_status = None
                self.stop()
                if self.nav_purpose == 'explore':
                    self.nav_purpose = None
                    self.scan_buffer.clear()
                    self.scan = None
                    self.last_segments = []
                    self.state = 'COLLECT'
                    self.state_deadline = time.time() + 2.0
                else:
                    self.nav_purpose = None
                    self.state = 'SETTLE'
                    self.state_deadline = time.time() + self.post_nav_pause_sec

            elif self.nav_status == 'failed':
                self.nav_status = None
                if self.nav_purpose == 'explore':
                    self.nav_purpose = None
                    self.state = 'COLLECT'
                    self.state_deadline = time.time() + self.startup_scan_sec
                else:
                    self.nav_purpose = None
                    self.state = 'FOLLOW'


        elif self.state == 'SETTLE':
            self.get_logger().info('I am in the SETTLE state', throttle_duration_sec=1.0)
            self.stop()
            if time.time() >= self.state_deadline:
                self.refresh_locked_wall()
                self.state = 'FOLLOW'
        
        elif self.state == 'FOLLOW':
            self.get_logger().info('I am in the FOLLOW state', throttle_duration_sec=1.0)
            self.lidar_follow_step()

        elif self.state == 'PROBE':
            self.stop()
            # Intentionally do nothing here.
            # Wait for probe_result_cb() to transition back to COLLECT.
            pass
        self.publish_markers()


def main():
    rclpy.init()
    node = GapExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
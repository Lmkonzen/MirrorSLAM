#!/usr/bin/env python3
"""
UR3e probe action server.

Drives the arm home -> poke -> home. Watches the gravity-compensated TCP
wrench during the poke and cancels the motion the moment force/torque
exceed threshold — that's how mirrors get distinguished from real walls.

Wrench thresholds are runtime-tunable:
  ros2 param set /probe_server force_threshold_n  <N>
  ros2 param set /probe_server torque_threshold_nm <Nm>

To calibrate from scratch: set thresholds absurdly high (e.g. 200N / 50Nm)
so the monitor never trips, do one free-air poke, read the peak |F| / |T|
that the disarm log prints, and set thresholds to ~1.5x those peaks.
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, JointConstraint, Constraints, MoveItErrorCodes,
)
from gap_explorer_interfaces.action import ProbeArm


HOME_JOINTS = [0.02, -1.57,  0.0,  -1.39, 1.47, 0.0]
POKE_JOINTS = [0.00, -3.0,   0.0,   0.0,  1.5,  0.3]

# Used as the detour pose when home is unreachable from a stuck position.
# Currently identical to HOME — fine for testing on a clear platform, but
# if Recovery 3 ever actually fires you'll want to set this to something
# noticeably "up and away" (e.g. shoulder_lift around -1.0, elbow folded).
STAGE_JOINTS = [0.02, -1.57,  0.0,  -1.39, 1.47, 0.0]

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class ArmProbeServer(Node):
    def __init__(self):
        super().__init__('probe_server')
        cb = ReentrantCallbackGroup()

        # ---- parameters ----
        self.declare_parameter('move_action_topic', '/move_action')
        # Unfiltered wrench — wrench_filtered adds enough latency that the UR
        # protective stop fires before the spike is published, so we never
        # see it in the topic.
        self.declare_parameter(
            'wrench_topic',
            '/force_torque_sensor_broadcaster/wrench'
        )
        self.declare_parameter('unlock_service',
                               '/dashboard_client/unlock_protective_stop')

        # Defaults are conservative starting points; calibrate per setup.
        self.declare_parameter('force_threshold_n', 3.0)
        self.declare_parameter('torque_threshold_nm', 0.3)
        # Joint-effort fallback. Wrench is the primary signal; this catches
        # the rare case where wrench publishing dies but joints are loaded.
        self.declare_parameter(
            'effort_thresholds',
            [4.0, 6.0, 4.0, 2.0, 2.0, 2.0]
        )
        self.declare_parameter('contact_consecutive', 1)

        move_topic   = self.get_parameter('move_action_topic').value
        wrench_topic = self.get_parameter('wrench_topic').value
        unlock_topic = self.get_parameter('unlock_service').value

        self.force_thresh        = float(self.get_parameter('force_threshold_n').value)
        self.torque_thresh       = float(self.get_parameter('torque_threshold_nm').value)
        self.effort_thresh       = np.array(
            self.get_parameter('effort_thresholds').value, dtype=float
        )
        self.contact_consecutive = int(self.get_parameter('contact_consecutive').value)

        # ---- MoveIt client ----
        self._move_client = ActionClient(self, MoveGroup, move_topic, callback_group=cb)
        self.get_logger().info(f'Waiting for {move_topic}...')
        self._move_client.wait_for_server()  # blocks until MoveGroup is up
        self.get_logger().info('Connected.')

        # ---- contact monitor state ----
        self._mon_lock = threading.Lock()
        self._monitor_active = False
        self._monitor_triggered = False
        self._monitor_reason = ''
        self._above_count_wrench = 0
        self._above_count_effort = 0
        self._peak_force = 0.0
        self._peak_torque = 0.0
        self._peak_effort_per_joint = np.zeros(6, dtype=float)

        self.create_subscription(
            WrenchStamped, wrench_topic,
            self._wrench_cb, qos_profile_sensor_data,
            callback_group=cb,
        )
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, qos_profile_sensor_data,
            callback_group=cb,
        )

        # ---- dashboard unlock service client ----
        # Best-effort. If we cancel a goal cleanly via the contact monitor
        # the protective stop won't fire and this is unused. Only matters
        # when the UR's hardware safety beat us to the punch.
        self._unlock_client = self.create_client(
            Trigger, unlock_topic, callback_group=cb
        )

        # ---- action server ----
        self._action_server = ActionServer(
            self, ProbeArm, 'probe_arm',
            execute_callback=self._execute,
            callback_group=cb,
        )

        # live param updates — see _on_params_changed
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info(
            f'probe_arm ready  '
            f'(F<={self.force_thresh:.2f}N, T<={self.torque_thresh:.2f}Nm, '
            f'consec={self.contact_consecutive}, wrench={wrench_topic})'
        )

    # ---- live parameter callback ----
    # Without this, ros2 param set updates the parameter store but the
    # cached self.force_thresh etc never change. Easy mistake to make.
    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'force_threshold_n':
                self.force_thresh = float(p.value)
                self.get_logger().info(f'force_threshold_n -> {self.force_thresh:.2f}N')
            elif p.name == 'torque_threshold_nm':
                self.torque_thresh = float(p.value)
                self.get_logger().info(f'torque_threshold_nm -> {self.torque_thresh:.2f}Nm')
            elif p.name == 'effort_thresholds':
                self.effort_thresh = np.array(p.value, dtype=float)
                self.get_logger().info(f'effort_thresholds -> {self.effort_thresh.tolist()}')
            elif p.name == 'contact_consecutive':
                self.contact_consecutive = int(p.value)
                self.get_logger().info(f'contact_consecutive -> {self.contact_consecutive}')
        return SetParametersResult(successful=True)

    # ---- contact monitor ----

    def _trip(self, reason: str):
        if self._monitor_triggered:
            return
        self._monitor_triggered = True
        self._monitor_reason = reason

    def _wrench_cb(self, msg):
        # Primary contact signal. The UR controller's wrench is already
        # gravity- and inertia-compensated, so anything above ~1N on a slow
        # poke is genuine contact, not motion noise.
        f = msg.wrench.force
        t = msg.wrench.torque
        fmag = float(np.sqrt(f.x*f.x + f.y*f.y + f.z*f.z))
        tmag = float(np.sqrt(t.x*t.x + t.y*t.y + t.z*t.z))
        with self._mon_lock:
            if not self._monitor_active or self._monitor_triggered:
                return
            if fmag > self._peak_force:  self._peak_force = fmag
            if tmag > self._peak_torque: self._peak_torque = tmag
            if fmag > self.force_thresh or tmag > self.torque_thresh:
                self._above_count_wrench += 1
                if self._above_count_wrench >= self.contact_consecutive:
                    self._trip(
                        f'wrench |F|={fmag:.2f}N |T|={tmag:.3f}Nm '
                        f'(F>{self.force_thresh:.2f} or T>{self.torque_thresh:.2f})'
                    )
            else:
                self._above_count_wrench = 0

    def _joint_state_cb(self, msg):
        # Backup signal. Joint efforts are noisier than the TCP wrench but
        # the topic is always there even if the F/T broadcaster hiccups.
        eff_map = {n: e for n, e in zip(msg.name, msg.effort)}
        eff = np.array([eff_map.get(n, 0.0) for n in JOINT_NAMES], dtype=float)
        with self._mon_lock:
            if not self._monitor_active or self._monitor_triggered:
                return
            abs_eff = np.abs(eff)
            self._peak_effort_per_joint = np.maximum(
                self._peak_effort_per_joint, abs_eff
            )
            over = abs_eff > self.effort_thresh
            if np.any(over):
                self._above_count_effort += 1
                if self._above_count_effort >= self.contact_consecutive:
                    j = int(np.argmax(abs_eff - self.effort_thresh))
                    self._trip(
                        f'joint {JOINT_NAMES[j]} effort {eff[j]:.2f}Nm '
                        f'> {self.effort_thresh[j]:.2f}Nm'
                    )
            else:
                self._above_count_effort = 0

    def _arm_monitor(self):
        with self._mon_lock:
            self._monitor_active = True
            self._monitor_triggered = False
            self._monitor_reason = ''
            self._above_count_wrench = 0
            self._above_count_effort = 0
            self._peak_force = 0.0
            self._peak_torque = 0.0
            self._peak_effort_per_joint = np.zeros(6, dtype=float)

    def _disarm_monitor(self):
        # Logs the peak readings the monitor saw — this is how you
        # calibrate the thresholds (see module docstring).
        with self._mon_lock:
            self._monitor_active = False
            peak_eff = ', '.join(
                f'{n}={v:.2f}' for n, v in zip(JOINT_NAMES, self._peak_effort_per_joint)
            )
            self.get_logger().info(
                f'monitor disarmed  '
                f'peak |F|={self._peak_force:.2f}N peak |T|={self._peak_torque:.3f}Nm  '
                f'(F_thresh={self.force_thresh:.2f}, T_thresh={self.torque_thresh:.2f})  '
                f'peak joint effort: {peak_eff}'
            )

    def _was_triggered(self):
        with self._mon_lock:
            return self._monitor_triggered, self._monitor_reason

    # ---- protective-stop unlock (best-effort) ----
    # The UR latches into a protective-stop state when its own safety
    # threshold trips. Until something calls this service, every
    # subsequent MoveGroup goal will fail with CONTROL_FAILED. We try the
    # unlock before recovery so the recovery has a chance of working.
    def _unlock_protective_stop(self):
        if not self._unlock_client.service_is_ready():
            return  # service not exposed by the driver — skip silently
        try:
            future = self._unlock_client.call_async(Trigger.Request())
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if done.wait(timeout=3.0):
                self.get_logger().info('Protective-stop unlock requested')
        except Exception as e:
            self.get_logger().warn(f'Unlock attempt failed: {e}')

    # ---- motion primitive ----

    def _move(self, name, joints,
              attempts=10, planning_time=5.0,
              tolerance=0.01, vel_scale=0.1, wait_s=30.0,
              monitor_contact=False):
        """Send a joint-space MoveGroup goal and block until it finishes.

        Returns (ok, code_or_str). On a contact-monitor trip the goal is
        cancelled and we return (False, 'contact').
        """
        request = MotionPlanRequest()
        request.group_name = 'ur_manipulator'
        request.num_planning_attempts = attempts
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = vel_scale
        request.max_acceleration_scaling_factor = vel_scale

        constraints = Constraints()
        for jname, jval in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jval)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        # holders so the inner callbacks can write back to this scope
        done_event = threading.Event()
        result_holder = [None]
        gh_holder = [None]

        def result_cb(future):
            result_holder[0] = future.result()
            done_event.set()

        def goal_response_cb(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'{name}: goal rejected')
                done_event.set()
                return
            gh_holder[0] = gh
            gh.get_result_async().add_done_callback(result_cb)

        if monitor_contact:
            self._arm_monitor()

        self._move_client.send_goal_async(goal_msg).add_done_callback(goal_response_cb)

        # Poll loop instead of a single wait so we can react to the contact
        # monitor mid-motion. 20 ms is fine — the monitor is what's fast.
        poll_dt = 0.02
        elapsed = 0.0
        contact_triggered = False
        while elapsed < wait_s:
            if done_event.wait(timeout=poll_dt):
                break
            elapsed += poll_dt
            if monitor_contact:
                tripped, reason = self._was_triggered()
                if tripped:
                    contact_triggered = True
                    self.get_logger().info(f'{name}: contact — {reason}')
                    if gh_holder[0] is not None:
                        gh_holder[0].cancel_goal_async()
                    done_event.wait(timeout=2.0)  # let the cancel land
                    break

        if monitor_contact:
            self._disarm_monitor()

        if contact_triggered:
            return False, 'contact'

        if result_holder[0] is None:
            self.get_logger().error(f'{name}: timed out')
            return False, None

        code = result_holder[0].result.error_code.val
        ok = (code == MoveItErrorCodes.SUCCESS)
        if not ok:
            self.get_logger().warn(f'{name}: MoveIt error {code}')
        return ok, code

    # ---- robust return-home ----
    # When a poke ends in collision, the arm is left at a pose MoveIt
    # considers "in collision," and plan-from-current returns
    # START_STATE_IN_COLLISION (-10). Three layers of progressively
    # uglier recovery handle this; in practice strategy 1 covers the
    # common case and we rarely see 2 or 3.
    def _safe_return_home(self) -> bool:
        # cheap; no-op if we never tripped a protective stop
        self._unlock_protective_stop()

        self.get_logger().info('Recovery 1/3: direct return home')
        ok, _ = self._move('return_home', HOME_JOINTS)
        if ok:
            return True

        self.get_logger().warn('Recovery 2/3: relaxed return home')
        ok, _ = self._move('return_home_relaxed', HOME_JOINTS,
                           attempts=25, planning_time=10.0,
                           tolerance=0.05, vel_scale=0.05, wait_s=45.0)
        if ok:
            return True

        # Strategy 3 only earns its keep if STAGE_JOINTS is genuinely
        # different from HOME (it isn't yet — see top of file).
        self.get_logger().warn('Recovery 3/3: via stage pose')
        ok, _ = self._move('stage', STAGE_JOINTS,
                           attempts=25, planning_time=10.0,
                           tolerance=0.05, vel_scale=0.05, wait_s=45.0)
        if not ok:
            self.get_logger().error('Could not reach stage pose; arm stuck')
            return False
        ok, _ = self._move('return_home_via_stage', HOME_JOINTS,
                           attempts=25, planning_time=10.0,
                           tolerance=0.05, vel_scale=0.05, wait_s=45.0)
        return ok

    # ---- action callback ----

    def _execute(self, goal_handle):
        feedback = ProbeArm.Feedback()
        result = ProbeArm.Result()

        feedback.state = 'homing'
        goal_handle.publish_feedback(feedback)
        ok, _ = self._move('home', HOME_JOINTS)
        if not ok:
            result.success = False
            result.object_detected = False
            result.message = 'Failed to reach home pose'
            goal_handle.abort(result)
            return result

        # Slow vel_scale during the poke — momentum scales with v^2 so a
        # halved speed gives 4x cleaner contact reading.
        feedback.state = 'poking'
        goal_handle.publish_feedback(feedback)
        poke_ok, poke_code = self._move(
            'poke', POKE_JOINTS,
            vel_scale=0.05,
            monitor_contact=True,
        )

        feedback.state = 'returning'
        goal_handle.publish_feedback(feedback)
        return_ok = self._safe_return_home()

        feedback.state = 'done'
        goal_handle.publish_feedback(feedback)

        if not return_ok:
            result.success = False
            result.object_detected = (not poke_ok)
            result.message = (
                f'Arm did not recover to home '
                f'(poke_code={poke_code}); manual intervention required'
            )
            goal_handle.abort(result)
            return result

        # Probe semantics: contact detected => a physical surface (mirror,
        # window, or wall) is at the lidar's perceived wall position. No
        # contact at full extension => the lidar saw a phantom and the
        # space is actually open. gap_explorer feeds object_detected=True
        # results into /detected_mirrors for the Nav2 costmap.
        if not poke_ok:
            via = 'contact monitor' if poke_code == 'contact' \
                else f'MoveIt code {poke_code}'
            result.success = True
            result.object_detected = True
            result.message = f'Poke blocked ({via}); surface detected'
        else:
            result.success = True
            result.object_detected = False
            result.message = 'Probe completed successfully'

        goal_handle.succeed(result)
        return result


def main():
    rclpy.init()
    node = ArmProbeServer()
    # MultiThreaded so the wrench/joint callbacks can fire while _move() is
    # blocked in its poll loop. Single-threaded would deadlock here.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
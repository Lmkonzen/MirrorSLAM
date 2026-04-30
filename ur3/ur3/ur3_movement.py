#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, JointConstraint, Constraints, MoveItErrorCodes,
)
from gap_explorer_interfaces.action import ProbeArm

HOME_JOINTS = [0.02, -1.57, 0.0, -1.39, 1.47, 0.0]
POKE_JOINTS = [0.00, -3, 0.00, 0.00, 1.5, 0.3]
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class ArmProbeServer(Node):
    def __init__(self):
        super().__init__('probe_server')
        cb = ReentrantCallbackGroup()

        # When the UR stack runs inside the /ur3e namespace (see
        # ur_isolated.launch.py), MoveIt's action server lives at
        # /ur3e/move_action.  This parameter lets callers override it
        # for setups that don't use the namespace.
        self.declare_parameter('move_action_topic', '/move_action')
        move_topic = self.get_parameter('move_action_topic').value

        self._move_client = ActionClient(self, MoveGroup, move_topic,
                                         callback_group=cb)
        self.get_logger().info(f'Waiting for {move_topic}...')
        self._move_client.wait_for_server()
        self.get_logger().info('Connected.')

        self._action_server = ActionServer(
            self, ProbeArm, 'probe_arm',
            execute_callback=self._execute,
            callback_group=cb,
        )
        self.get_logger().info('probe_arm action server ready.')

    def _move(self, name, joints):
        """Block until the joint move completes. Returns True on success."""
        import threading

        request = MotionPlanRequest()
        request.group_name = 'ur_manipulator'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        constraints = Constraints()
        for jname, jval in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(jval)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        done_event = threading.Event()
        result_holder = [None]

        def result_cb(future):
            result_holder[0] = future.result()
            done_event.set()

        def goal_response_cb(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'{name}: goal rejected')
                done_event.set()
                return
            result_future = gh.get_result_async()
            result_future.add_done_callback(result_cb)

        send_future = self._move_client.send_goal_async(goal_msg)
        send_future.add_done_callback(goal_response_cb)

        # Block this thread (the action executor thread) until motion completes.
        # The MultiThreadedExecutor keeps running other callbacks on other threads.
        done_event.wait(timeout=30.0)

        if result_holder[0] is None:
            self.get_logger().error(f'{name}: timed out or rejected')
            return False

        ok = result_holder[0].result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().error(f'{name}: MoveIt error {result_holder[0].result.error_code.val}')
        return ok

    def _execute(self, goal_handle):
        feedback = ProbeArm.Feedback()
        result = ProbeArm.Result()

        # Move to home first (safe starting pose)
        feedback.state = 'homing'
        goal_handle.publish_feedback(feedback)
        if not self._move('Home', HOME_JOINTS):
            result.success = False
            result.object_detected = False
            result.message = 'Failed to reach home pose'
            goal_handle.abort(result)
            return result

        # Poke
        feedback.state = 'poking'
        goal_handle.publish_feedback(feedback)
        poke_ok = self._move('Poke', POKE_JOINTS)

        # Always try to return home
        feedback.state = 'returning'
        goal_handle.publish_feedback(feedback)
        self._move('Return home', HOME_JOINTS)

        if not poke_ok:
            # Planning/execution failure on poke = something blocked the arm
            result.success = False
            result.object_detected = True
            result.message = 'Poke motion failed — obstacle likely detected'
        else:
            result.success = True
            result.object_detected = False
            result.message = 'Probe completed successfully'

        feedback.state = 'done'
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed(result)
        return result


def main():
    rclpy.init()
    node = ArmProbeServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
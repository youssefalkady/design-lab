"""DrawSquare action server.

Calls the reset service for a clean screen and a known start pose, then sends
exactly 4 goals to the goToPose action server (one per corner), waiting for
each to finish before sending the next.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim_msgs.msg import Pose

from turtle_controller.geometry import normalize_angle
from turtle_controller.ros_utils import wait_for_future
from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose

SIDES = 4


class SquareServer(Node):

    def __init__(self):
        super().__init__('square_server')

        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('default_side_length', 2.0)
        self.declare_parameter('workspace_min', 0.5)
        self.declare_parameter('workspace_max', 10.5)
        self.declare_parameter('server_timeout', 5.0)

        group = ReentrantCallbackGroup()
        turtle = self.get_parameter('turtle_name').value
        self._pose = None
        self._pose_count = 0
        self.create_subscription(
            Pose, f'/{turtle}/pose', self._pose_callback, 10, callback_group=group)
        self._reset_client = self.create_client(
            Trigger, 'reset_turtle', callback_group=group)
        self._go_to_pose = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=group)

        self._action_server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('square server ready on /draw_square')

    def _param(self, name):
        return self.get_parameter(name).value

    def _pose_callback(self, msg):
        self._pose = msg
        self._pose_count += 1

    def _goal_callback(self, goal_request):
        side = goal_request.side_length
        if not math.isfinite(side):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        side = goal_handle.request.side_length
        if side <= 0.0:
            side = self._param('default_side_length')
        self.get_logger().info(f'drawing square with side {side:.3f}')

        try:
            x0, y0 = self._reset()
        except RuntimeError as e:
            goal_handle.abort()
            return self._result(False, 0, str(e))

        corners = self._square_corners(x0, y0, side)
        lo, hi = self._param('workspace_min'), self._param('workspace_max')
        if not all(lo <= c.x <= hi and lo <= c.y <= hi for c in corners):
            goal_handle.abort()
            return self._result(
                False, 0, f'square with side {side} leaves the workspace [{lo}, {hi}]')

        for i, corner in enumerate(corners, start=1):
            outcome = self._drive_to_corner(goal_handle, i, corner)
            if outcome == 'canceled':
                goal_handle.canceled()
                self.get_logger().info(f'square canceled on side {i}')
                return self._result(False, i - 1, 'canceled')
            if outcome != 'succeeded':
                goal_handle.abort()
                return self._result(False, i - 1, f'side {i}: {outcome}')

        goal_handle.succeed()
        self.get_logger().info(f'square done ({SIDES} goToPose goals sent)')
        return self._result(True, SIDES, 'square completed')

    def _reset(self):
        """Call the reset service and return the start position."""
        timeout = self._param('server_timeout')
        if not self._reset_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('reset_turtle service not available')
        future = self._reset_client.call_async(Trigger.Request())
        if not wait_for_future(future, timeout) or not future.result().success:
            raise RuntimeError('reset_turtle call failed')

        # Wait for a few fresh pose messages so we read the post-teleport pose.
        seen = self._pose_count
        deadline = time.monotonic() + timeout
        while self._pose_count < seen + 3:
            if time.monotonic() > deadline:
                raise RuntimeError('no pose received after reset')
            time.sleep(0.01)
        return self._pose.x, self._pose.y

    @staticmethod
    def _square_corners(x0, y0, side):
        """Counter-clockwise corners. Each corner's yaw faces the next side,
        so every side starts already aligned and is drawn as a straight line."""
        offsets = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
        return [
            TargetPose(x=x0 + dx, y=y0 + dy,
                       yaw=normalize_angle(i * math.pi / 2.0))
            for i, (dx, dy) in enumerate(offsets, start=1)
        ]

    def _drive_to_corner(self, goal_handle, side, corner):
        timeout = self._param('server_timeout')
        if not self._go_to_pose.wait_for_server(timeout_sec=timeout):
            return 'go_to_pose server not available'

        def forward_feedback(msg):
            fb = DrawSquare.Feedback(
                current_side=side, current_target=corner,
                remaining_distance=msg.feedback.remaining_distance)
            goal_handle.publish_feedback(fb)

        self.get_logger().info(
            f'side {side}/{SIDES}: goToPose -> ({corner.x:.3f}, '
            f'{corner.y:.3f}, {corner.yaw:.3f})')
        send_future = self._go_to_pose.send_goal_async(
            GoToPose.Goal(target=corner), feedback_callback=forward_feedback)
        if not wait_for_future(send_future, timeout):
            return 'goToPose goal was not answered'
        child = send_future.result()
        if not child.accepted:
            return 'goToPose goal rejected'

        result_future = child.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                wait_for_future(child.cancel_goal_async(), timeout)
                wait_for_future(result_future, timeout)
                return 'canceled'
            time.sleep(0.02)

        result = result_future.result().result
        return 'succeeded' if result.success else result.message

    @staticmethod
    def _result(success, sides, message):
        return DrawSquare.Result(
            success=success, sides_completed=sides, message=message)


def main(args=None):
    rclpy.init(args=args)
    node = SquareServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""goToPose action server.

Drives the turtle to a target (x, y, yaw) using the State machine in
``motion_states``. It owns the speed-limit parameters and the service that
changes them; the limits are re-read on every control cycle so changes take
effect while the turtle is moving.
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim_msgs.msg import Pose

from turtle_controller.motion_states import ControlContext, RotateToTarget
from turtle_interfaces.action import GoToPose
from turtle_interfaces.msg import TargetPose
from turtle_interfaces.srv import SetSpeedLimits

# Parameters that must stay strictly positive.
POSITIVE_PARAMS = (
    'max_linear_speed', 'max_angular_speed', 'k_linear', 'k_angular',
    'position_tolerance', 'yaw_tolerance', 'heading_tolerance',
    'control_rate', 'feedback_rate', 'goal_timeout',
)


class GoToPoseServer(Node):

    def __init__(self):
        super().__init__('go_to_pose_server')

        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter(
            'max_linear_speed', 2.0,
            ParameterDescriptor(description='Max linear speed [m/s], > 0'))
        self.declare_parameter(
            'max_angular_speed', 2.0,
            ParameterDescriptor(description='Max angular speed [rad/s], > 0'))
        self.declare_parameter('k_linear', 1.5)
        self.declare_parameter('k_angular', 4.0)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', 0.01)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('feedback_rate', 10.0)
        self.declare_parameter('goal_timeout', 60.0)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._log_parameters)

        turtle = self.get_parameter('turtle_name').value
        self._pose = None
        self._pose_sub = self.create_subscription(
            Pose, f'/{turtle}/pose', self._pose_callback, 10)
        self._cmd_pub = self.create_publisher(Twist, f'/{turtle}/cmd_vel', 10)

        self._speed_srv = self.create_service(
            SetSpeedLimits, 'set_speed_limits', self._set_speed_limits_callback)

        self._busy = False
        self._busy_lock = threading.Lock()
        self._goals_received = 0
        self._action_server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'goToPose server ready (turtle: {turtle}, '
            f'max_linear_speed={self._param("max_linear_speed")}, '
            f'max_angular_speed={self._param("max_angular_speed")})')

    # ----------------------------------------------------------- parameters
    def _param(self, name):
        return self.get_parameter(name).value

    def _validate_parameters(self, params):
        for p in params:
            if p.name not in POSITIVE_PARAMS:
                continue
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be a number')
            if p.value <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be > 0 (got {p.value})')
        return SetParametersResult(successful=True)

    def _log_parameters(self, params):
        for p in params:
            self.get_logger().info(f'parameter {p.name} = {p.value}')

    def _set_speed_limits_callback(self, request, response):
        results = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE,
                      float(request.max_linear_speed)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE,
                      float(request.max_angular_speed)),
        ])
        response.success = results.successful
        if results.successful:
            response.message = (
                f'max_linear_speed={request.max_linear_speed}, '
                f'max_angular_speed={request.max_angular_speed}')
        else:
            response.message = f'rejected: {results.reason}'
            self.get_logger().warning(f'set_speed_limits {response.message}')
        return response

    # ---------------------------------------------------------------- motion
    def _pose_callback(self, msg):
        self._pose = msg

    def _publish_cmd(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)

    def _control_context(self, target):
        pose = self._pose
        return ControlContext(
            x=pose.x, y=pose.y, theta=pose.theta,
            goal_x=target.x, goal_y=target.y, goal_yaw=target.yaw,
            max_linear=self._param('max_linear_speed'),
            max_angular=self._param('max_angular_speed'),
            k_linear=self._param('k_linear'),
            k_angular=self._param('k_angular'),
            position_tolerance=self._param('position_tolerance'),
            yaw_tolerance=self._param('yaw_tolerance'),
            heading_tolerance=self._param('heading_tolerance'))

    def _goal_callback(self, goal_request):
        t = goal_request.target
        with self._busy_lock:
            if self._busy:
                self.get_logger().warning(
                    f'rejecting goal ({t.x:.2f}, {t.y:.2f}, {t.yaw:.2f}): busy')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        try:
            return self._execute(goal_handle)
        finally:
            self._publish_cmd()
            with self._busy_lock:
                self._busy = False

    def _execute(self, goal_handle):
        target = goal_handle.request.target
        self._goals_received += 1
        self.get_logger().info(
            f'goal #{self._goals_received}: x={target.x:.3f} '
            f'y={target.y:.3f} yaw={target.yaw:.3f}')

        deadline = time.monotonic() + 2.0
        while self._pose is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._pose is None:
            goal_handle.abort()
            return self._result(target, False, 'no pose received from turtlesim')

        state = RotateToTarget()
        start = time.monotonic()
        last_feedback = 0.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_cmd()
                goal_handle.canceled()
                self.get_logger().info('goal canceled')
                return self._result(target, False, 'canceled')
            if time.monotonic() - start > self._param('goal_timeout'):
                self._publish_cmd()
                goal_handle.abort()
                self.get_logger().warning('goal timed out')
                return self._result(target, False, 'timed out')

            ctx = self._control_context(target)
            step = state.step(ctx)
            state = step.next_state
            if state.done:
                break
            self._publish_cmd(step.linear, step.angular)

            now = time.monotonic()
            if now - last_feedback >= 1.0 / self._param('feedback_rate'):
                last_feedback = now
                feedback = GoToPose.Feedback()
                feedback.current_pose = TargetPose(x=ctx.x, y=ctx.y, yaw=ctx.theta)
                feedback.remaining_distance = ctx.distance_error
                feedback.remaining_angle = step.remaining_angle
                feedback.phase = state.name
                goal_handle.publish_feedback(feedback)

            time.sleep(1.0 / self._param('control_rate'))

        self._publish_cmd()
        goal_handle.succeed()
        result = self._result(target, True, 'target reached')
        self.get_logger().info(
            f'goal #{self._goals_received} reached: position_error='
            f'{result.position_error:.4f} yaw_error={result.yaw_error:.4f}')
        return result

    def _result(self, target, success, message):
        result = GoToPose.Result(success=success, message=message)
        if self._pose is not None:
            ctx = self._control_context(target)
            result.final_pose = TargetPose(x=ctx.x, y=ctx.y, yaw=ctx.theta)
            result.position_error = ctx.distance_error
            result.yaw_error = ctx.yaw_error
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
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

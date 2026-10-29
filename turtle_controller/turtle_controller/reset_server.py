"""Reset service: clear the screen and teleport the turtle to its start pose.

Uses only turtlesim's own services (set_pen, teleport_absolute, clear),
through ``TurtlesimFacade``. The pen is lifted before teleporting so no line is
drawn from the old position to the start.
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from turtle_controller.turtlesim_facade import TurtlesimError, TurtlesimFacade


class ResetServer(Node):

    def __init__(self):
        super().__init__('reset_server')

        self.declare_parameter('turtle_name', 'turtle1')
        # turtlesim spawns turtle1 at the window centre, facing +x
        self.declare_parameter('start_x', 5.544445)
        self.declare_parameter('start_y', 5.544445)
        self.declare_parameter('start_theta', 0.0)
        # turtlesim default pen
        self.declare_parameter('pen_r', 179)
        self.declare_parameter('pen_g', 184)
        self.declare_parameter('pen_b', 255)
        self.declare_parameter('pen_width', 3)

        # Clients get their own group so their responses can be processed
        # while the service callback is waiting on them.
        self._turtlesim = TurtlesimFacade(
            self, self.get_parameter('turtle_name').value,
            callback_group=MutuallyExclusiveCallbackGroup())
        self._srv = self.create_service(
            Trigger, 'reset_turtle', self._reset_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info('reset service ready on /reset_turtle')

    def _param(self, name):
        return self.get_parameter(name).value

    def _reset_callback(self, request, response):
        x, y, theta = (self._param('start_x'), self._param('start_y'),
                       self._param('start_theta'))
        pen = (self._param('pen_r'), self._param('pen_g'),
               self._param('pen_b'), self._param('pen_width'))
        try:
            self._turtlesim.set_pen(*pen, off=True)
            self._turtlesim.teleport_absolute(x, y, theta)
            self._turtlesim.clear()
            self._turtlesim.set_pen(*pen, off=False)
        except TurtlesimError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'reset failed: {e}')
            return response
        response.success = True
        response.message = f'screen cleared, turtle at ({x:.3f}, {y:.3f}, {theta:.3f})'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetServer()
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

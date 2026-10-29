"""Facade over the turtlesim services used by this project.

Calling a ROS service takes a client, a request object, an availability check,
an async call and a wait. ``TurtlesimFacade`` hides that behind three plain
methods (set_pen, teleport_absolute, clear) so the reset logic reads as the
steps it performs.
"""

from std_srvs.srv import Empty
from turtlesim_msgs.srv import SetPen, TeleportAbsolute

from turtle_controller.ros_utils import wait_for_future


class TurtlesimError(RuntimeError):
    pass


class TurtlesimFacade:

    def __init__(self, node, turtle_name='turtle1', callback_group=None,
                 timeout_sec=5.0):
        self._node = node
        self._timeout = timeout_sec
        self._clear = node.create_client(
            Empty, '/clear', callback_group=callback_group)
        self._set_pen = node.create_client(
            SetPen, f'/{turtle_name}/set_pen', callback_group=callback_group)
        self._teleport = node.create_client(
            TeleportAbsolute, f'/{turtle_name}/teleport_absolute',
            callback_group=callback_group)

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request(r=r, g=g, b=b, width=width, off=1 if off else 0)
        self._call(self._set_pen, req)

    def teleport_absolute(self, x, y, theta):
        req = TeleportAbsolute.Request(x=float(x), y=float(y), theta=float(theta))
        self._call(self._teleport, req)

    def clear(self):
        self._call(self._clear, Empty.Request())

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise TurtlesimError(f'service {client.srv_name} is not available')
        future = client.call_async(request)
        if not wait_for_future(future, self._timeout):
            raise TurtlesimError(f'service {client.srv_name} timed out')
        if future.exception() is not None:
            raise TurtlesimError(f'service {client.srv_name} failed: {future.exception()}')
        return future.result()

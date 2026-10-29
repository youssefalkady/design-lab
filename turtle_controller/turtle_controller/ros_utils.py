"""Helpers for waiting on rclpy futures from inside callbacks.

Waiting is done by polling so it is safe from any executor thread, as long as
the node is spun by a MultiThreadedExecutor and the clients live in a callback
group that can run in parallel with the waiting callback.
"""

import time


def wait_for_future(future, timeout_sec=None, poll_period=0.01) -> bool:
    """Block until future is done. Returns False on timeout."""
    deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
    while not future.done():
        if deadline is not None and time.monotonic() > deadline:
            return False
        time.sleep(poll_period)
    return True

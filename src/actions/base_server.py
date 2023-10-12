import queue
from typing import Union


class ActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: Union[queue.Queue, None] = None,
    ):
        self.server_name = server_name

    def execute(self, goal):
        pass

    def put_feedback(self, feedback=None):
        pass

    def put_result(self, result=None):
        pass

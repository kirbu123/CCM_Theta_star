import queue
from typing import Union

import actionlib
import rospy
from communication_msgs.msg import (
    MoveToAction,
    MoveToFeedback,
    MoveToGoal,
    MoveToResult,
    OpenSeeDSetterAction,
    OpenSeeDSetterGoal,
    PickupObjectAction,
    PickupObjectGoal,
    PickupObjectResult,
    PutObjectAction,
    PutObjectGoal,
    PutObjectResult,
)


class ActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: Union[queue.Queue, None] = None,
    ):
        self.server_name = server_name

    def execute(self, goal: Union[MoveToGoal, PickupObjectGoal, PutObjectGoal, OpenSeeDSetterGoal]):
        pass

    def put_feedback(self, feedback: Union[MoveToFeedback, None] = None):
        pass

    def put_result(self, result: Union[MoveToResult, PickupObjectResult, PutObjectResult, None] = None):
        pass


class MoveToActionServer(ActionServer):
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        super().__init__(server_name, task_queue)

        self.server = actionlib.SimpleActionServer(server_name, MoveToAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self._feedback_queue = queue.Queue()
        self._result_queue = queue.Queue()

    def put_feedback(self, feedback: MoveToFeedback):
        self._feedback_queue.put(feedback)

    def put_result(self, result: MoveToResult):
        self._result_queue.put(result)

    def execute(self, goal: MoveToGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        while self._result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            feedback = self._feedback_queue.get()
            # print(f'[{self.server_name}] Sending feedback:', feedback.__str__())
            self.server.publish_feedback(feedback)
            self._feedback_queue.task_done()

        result = self._result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self._result_queue.task_done()


class PickupObjectActionServer(ActionServer):
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        super().__init__(server_name, task_queue)

        self.server = actionlib.SimpleActionServer(server_name, PickupObjectAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self._feedback_queue = None  # queue.Queue()
        self._result_queue = queue.Queue()

    def put_result(self, result: PickupObjectResult):
        self._result_queue.put(result)

    def execute(self, goal: PickupObjectGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        print(f"[{self.server_name} action server] Waiting for result...")
        while self._result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            # feedback = self._feedback_queue.get()
            # self.server.publish_feedback(feedback)
            # self._feedback_queue.task_done()

        result = self._result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self._result_queue.task_done()


class PutObjectActionServer(ActionServer):
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        super().__init__(server_name, task_queue)

        self.server = actionlib.SimpleActionServer(server_name, PutObjectAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self._feedback_queue = None  # queue.Queue()
        self._result_queue = queue.Queue()

    def put_result(self, result: PutObjectResult):
        self._result_queue.put(result)

    def execute(self, goal: PutObjectGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        print(f"[{self.server_name}] Waiting for result...")
        while self._result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            # feedback = self._feedback_queue.get()
            # self.server.publish_feedback(feedback)
            # self._feedback_queue.task_done()

        result = self._result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self._result_queue.task_done()


class OpenSeedActionServer(ActionServer):
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        super().__init__(server_name, task_queue)
        self.server = actionlib.SimpleActionServer(
            "text_query_generation", OpenSeeDSetterAction, self.execute, False
        )
        self.server.start()
        self.task_queue = task_queue

        self._feedback_queue = None  # queue.Queue()
        self._result_queue = None  # queue.Queue()

    def execute(self, goal: OpenSeeDSetterGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        # while self._result_queue.empty():
        #     rospy.sleep(0.2)
        #     if self.server.is_preempt_requested():
        #         print(f'[{self.server_name}] The goal has been preempted')
        #         self.server.set_preempted()
        #         return
        #     print(f'[{self.server_name}] Waiting for result...')
        #     # feedback = self._feedback_queue.get()
        #     # self.server.publish_feedback(feedback)
        #     # self._feedback_queue.task_done()

        # result = self._result_queue.get()
        print(f"[{self.server_name}] Done!")
        self.server.set_succeeded()
        # self.server.set_succeeded(result)
        # self._result_queue.task_done()

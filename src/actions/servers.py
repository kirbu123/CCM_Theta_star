import queue

import actionlib
import rospy
from communication_msgs.msg import (
    MoveToAction,
    MoveToGoal,
    OpenSeeDSetterAction,
    OpenSeeDSetterGoal,
    PickupObjectAction,
    PickupObjectGoal,
    PutObjectAction,
    PutObjectGoal,
)


class MoveToActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        self.server_name = server_name

        self.server = actionlib.SimpleActionServer(server_name, MoveToAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self.feedback_queue = queue.Queue()
        self.result_queue = queue.Queue()

    def execute(self, goal: MoveToGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        while self.result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            feedback = self.feedback_queue.get()
            # print(f'[{self.server_name}] Sending feedback:', feedback.__str__())
            self.server.publish_feedback(feedback)
            self.feedback_queue.task_done()

        result = self.result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self.result_queue.task_done()


class PickupObjectActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        self.server_name = server_name

        self.server = actionlib.SimpleActionServer(server_name, PickupObjectAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self.feedback_queue = None  # queue.Queue()
        self.result_queue = queue.Queue()

    def execute(self, goal: PickupObjectGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        print(f"[{self.server_name} action server] Waiting for result...")
        while self.result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            # feedback = self.feedback_queue.get()
            # self.server.publish_feedback(feedback)
            # self.feedback_queue.task_done()

        result = self.result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self.result_queue.task_done()


class PutObjectActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        self.server_name = server_name

        self.server = actionlib.SimpleActionServer(server_name, PutObjectAction, self.execute, False)
        self.server.start()
        self.task_queue = task_queue

        self.feedback_queue = None  # queue.Queue()
        self.result_queue = queue.Queue()

    def execute(self, goal: PutObjectGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        print(f"[{self.server_name}] Waiting for result...")
        while self.result_queue.empty():
            rospy.sleep(0.2)
            if self.server.is_preempt_requested():
                print(f"[{self.server_name}] The goal has been preempted")
                self.server.set_preempted()
                return
            # feedback = self.feedback_queue.get()
            # self.server.publish_feedback(feedback)
            # self.feedback_queue.task_done()

        result = self.result_queue.get()
        print(f"[{self.server_name}] Sending result:", result.__str__())
        self.server.set_succeeded(result)
        self.result_queue.task_done()


class OpenSeedActionServer:
    def __init__(
        self,
        server_name: str,
        task_queue: queue.Queue,
    ):
        self.server_name = server_name

        self.server = actionlib.SimpleActionServer(
            "text_query_generation", OpenSeeDSetterAction, self.execute, False
        )
        self.server.start()
        self.task_queue = task_queue

        self.feedback_queue = None  # queue.Queue()
        self.result_queue = None  # queue.Queue()

    def execute(self, goal: OpenSeeDSetterGoal):
        task = (self.server_name, goal)  # server id, task
        print(f"Task added: (server_id, task) -- {task.__str__()}")
        self.task_queue.put(task)

        # while self.result_queue.empty():
        #     rospy.sleep(0.2)
        #     if self.server.is_preempt_requested():
        #         print(f'[{self.server_name}] The goal has been preempted')
        #         self.server.set_preempted()
        #         return
        #     print(f'[{self.server_name}] Waiting for result...')
        #     # feedback = self.feedback_queue.get()
        #     # self.server.publish_feedback(feedback)
        #     # self.feedback_queue.task_done()

        # result = self.result_queue.get()
        print(f"[{self.server_name}] Done!")
        self.server.set_succeeded()
        # self.server.set_succeeded(result)
        # self.result_queue.task_done()

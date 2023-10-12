import queue

from src.actions.base_server import ActionServer
from src.config import get_config

cfg = get_config()

if cfg.mode == "online":
    if cfg.ros == "noetic":
        import actionlib
        import rospy
    else:
        # Here we import ROS2 packages
        # TODO
        raise NotImplementedError("ROS2 Actions is not implemented yet")
        pass
    # Assume that interface is the same for both ROS1 and ROS2,
    # if not - place the imports in the if/else block
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

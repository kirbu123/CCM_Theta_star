import numpy as np
from communication_msgs.msg import (
    MoveToFeedback,
    MoveToResult,
    PickupObjectResult,
    PutObjectResult,
)
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from omni.usd import get_world_transform_matrix

from src.config import Config
from src.controllers.pick_place import PickPlaceController
from src.tasks.pick_place import PickPlace


class HuskyController:
    def __init__(self, cfg: Config, world) -> None:
        self.cfg = cfg
        self.pick_task = PickPlace(
            name="denso_pick_place",
            cfg=cfg,
        )
        self.world = world
        self.world.add_task(self.pick_task)
        self.world.reset()
        self.denso = self.world.scene.get_object("my_ur5")

        self.world.reset()
        self.husky = self.pick_task.robots["husky"]
        self.ur5 = self.pick_task.robots["ur5"]
        self.cube = self.pick_task.obj["cube"]

        self.scene = {}
        self.obj_on_hand = ""

        self.world.reset()

        self.prim_trans_point = self.pick_task.robots["trans_point"]

        self.husky_controller = WheelBasePoseController(
            name="cool_controller",
            open_loop_wheel_controller=DifferentialController(
                name="simple_control", wheel_radius=cfg.wheel_radius, wheel_base=cfg.wheel_base
            ),
            is_holonomic=False,
        )
        self.pick_place_controller = PickPlaceController(
            name="controller",
            robot_articulation=self.denso,
            gripper=self.denso.gripper,
            cfg=cfg,
        )

        self.task_params = self.world.get_task("denso_pick_place").get_params()
        self.articulation_controller = self.denso.get_articulation_controller()

    def move_to_location(self, task, action_server):
        stage = get_current_stage()

        _location = task.location.replace(" ", "_")
        _object = task.object.replace(" ", "_")

        obj_move_str = ""
        if _location != "unspecified":
            obj_move_str = _location
        elif _object != "unspecified":
            obj_move_str = _object
        else:
            return False
        print(f"obj_move_str: {str(obj_move_str)}")
        prim_obj_move = stage.GetPrimAtPath(f"/background/{obj_move_str}")
        pose_obj_move = get_world_transform_matrix(prim_obj_move)
        position_obj_move = pose_obj_move.ExtractTranslation()

        print(f"Now move to {obj_move_str}, position {position_obj_move.__repr__()}\n")

        distance = 100
        if (_object == "table") or (_object == "drawer"):
            stop_distance = 1.6
        else:
            stop_distance = 1.2
        while not (distance < stop_distance):
            # Sending feedback
            feedback = MoveToFeedback()
            feedback.x = position_obj_move[0]
            feedback.y = position_obj_move[1]
            action_server.feedback_queue.put(feedback)
            # \\\\\\

            position, orientation = self.husky.get_world_pose()
            # print(f"Distance to {obj_move_str}: {distance}\n")
            # sys.stdout.flush()
            # print(f"My position: {position[:2]}\n target position: {position_obj_move[:2]}\n")
            # print(f"Distance to {obj_move_str}: {distance}\n")
            wheel_actions = self.husky_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=position_obj_move[:2],
                lateral_velocity=self.cfg.lateral_velocity,
                yaw_velocity=self.cfg.yaw_velocity,
                position_tol=self.cfg.position_tol,
            )

            wheel_actions.joint_velocities = np.tile(wheel_actions.joint_velocities, 2)
            self.husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)
            distance = np.sum(
                (self.husky.get_world_pose()[0][:2] - position_obj_move[:2]) ** 2
            )  # Compute distance   between husky and target

        # Sending result
        result = MoveToResult()
        result.result = f"Done! Distance to goal {obj_move_str}: {distance}"
        action_server.result_queue.put(result)
        print(f"Result sent: {result}")

    def pickup_object(self, task, action_server):
        _object = task.object.replace(" ", "_")

        obj_pick_str = ""
        if _object != "unspecified":
            obj_pick_str = _object
        else:
            return False

        stage = get_current_stage()
        prim_obj_pick = stage.GetPrimAtPath(f"/background/{obj_pick_str}")
        pose_obj_pick = get_world_transform_matrix(prim_obj_pick)
        position_obj_pick = pose_obj_pick.ExtractTranslation()
        print(f"Now move to {obj_pick_str}, position {position_obj_pick}\n")

        position, orientation = self.husky.get_world_pose()

        for _ in range(10):
            wheel_actions = self.husky_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=position[:2],
                lateral_velocity=self.cfg.lateral_velocity,
                yaw_velocity=self.cfg.yaw_velocity,
                position_tol=self.cfg.position_tol,
            )
            wheel_actions.joint_velocities = np.tile(wheel_actions.joint_velocities, 2)
            self.husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

        position, orientation = self.husky.get_world_pose()
        self.pick_place_controller._cspace_controller.reset()

        while not self.pick_place_controller.pick_done():
            # wheel_actions=self.husky_controller.forward(start_position=position,
            #                                         start_orientation=orientation,
            #                                         goal_position=position[:2],
            #                                         lateral_velocity=self.cfg.lateral_velocity,
            #                                         yaw_velocity=self.cfg.yaw_velocity,
            #                                         position_tol = self.cfg.position_tol,)
            # wheel_actions.joint_velocities =np.tile(wheel_actions.joint_velocities, 2)
            # self.husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

            observations = self.world.get_observations()
            end_effector_offset = np.array(self.cfg.end_effector_offset) - np.array([0, 0, 0.0])  # For cup
            # end_effector_orientation =

            actions = self.pick_place_controller.forward(
                # picking_position=cube.get_world_pose()[0],
                picking_position=position_obj_pick,
                placing_position=observations[self.task_params["cube_name"]["value"]][
                    "target_position"
                ],  # # the placing position is not matter here, because only pick
                current_joint_positions=observations[self.task_params["robot_name"]["value"]][
                    "joint_positions"
                ],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=end_effector_offset,
                end_effector_orientation=euler_angles_to_quat(np.array([0, np.pi, 0.5 * np.pi])),
                prim_trans_point=self.prim_trans_point,
            )
            self.articulation_controller.apply_action(actions)
        # ee_pose = observations[task_params["robot_name"]["value"]]["end_effector_position"]

        self.pick_place_controller.pause()
        print("pick done!")
        result = PickupObjectResult()
        result.result = f"Done! Object {obj_pick_str} is on hand"
        action_server.result_queue.put(result)
        print(f"Result sent: {result}")

    def put_object(self, task, action_server):
        _location = task.location.replace(" ", "_")
        _object = task.object.replace(" ", "_")

        obj_put_str = ""
        obj_loc_str = ""
        if _object != "unspecified":
            obj_put_str = _object
        else:
            return False
        if _location != "unspecified":
            obj_loc_str = _location
        else:
            return False
        print(f"Now put {obj_put_str} on {obj_loc_str}\n")
        stage = get_current_stage()
        prim_obj_put_2 = stage.GetPrimAtPath(f"/background/{obj_loc_str}")
        pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
        position_obj_2 = pose_obj_2.ExtractTranslation()
        if (obj_loc_str == "table") or (obj_loc_str == "drawer"):
            prim_obj_put_2 = stage.GetPrimAtPath(f"/World/{obj_loc_str}/point")
            pose_obj_2 = get_world_transform_matrix(prim_obj_put_2)
            position_obj_2 = pose_obj_2.ExtractTranslation()
            position_obj_2[-1] = (
                position_obj_2[-1] + 0.036 if obj_loc_str == "table" else position_obj_2[-1] + 0.06
            )  # offset of place
            print(f"position_obj_loc: {position_obj_2}\n")
        else:
            position_obj_2[-1] = position_obj_2[-1] + 0.4

        position, orientation = self.husky.get_world_pose()

        for _ in range(10):
            wheel_actions = self.husky_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=position[:2],
                lateral_velocity=self.cfg.lateral_velocity,
                yaw_velocity=self.cfg.yaw_velocity,
                position_tol=self.cfg.position_tol,
            )
            wheel_actions.joint_velocities = np.tile(wheel_actions.joint_velocities, 2)
            self.husky.apply_wheel_actions(wheel_actions)
            self.world.step(render=True)

        position, orientation = self.husky.get_world_pose()
        self.pick_place_controller._cspace_controller.reset()

        self.pick_place_controller.resume()
        print(f"is done: {self.pick_place_controller.is_done()}")
        while not self.pick_place_controller.is_done():
            self.world.step(render=True)

            observations = self.world.get_observations()
            end_effector_orientation = (
                euler_angles_to_quat(np.array([0, np.pi, 0.5 * np.pi]))
                if obj_loc_str == "table"
                else euler_angles_to_quat(np.array([0, np.pi, 0.8 * np.pi]))
            )

            actions = self.pick_place_controller.forward(
                picking_position=self.cube.get_world_pose()[0],
                # placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
                placing_position=position_obj_2,
                current_joint_positions=observations[self.task_params["robot_name"]["value"]][
                    "joint_positions"
                ],
                # end_effector_offset=np.array([0, 0, 0.25]),
                end_effector_offset=np.array(self.cfg.end_effector_offset),
                end_effector_orientation=end_effector_orientation,
                prim_trans_point=self.prim_trans_point,
                ee_pose=observations[self.task_params["robot_name"]["value"]]["end_effector_position"],
            )

            self.articulation_controller.apply_action(actions)
        self.pick_place_controller.reset()
        result = PutObjectResult()
        result.result = f"Done! Object {obj_put_str} is on hand"
        action_server.result_queue.put(result)
        print(f"Result sent: {result}")

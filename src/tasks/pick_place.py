# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from omni.isaac.core.utils.nucleus import get_assets_root_path
from typing import Optional
import numpy as np
import os

from omni.isaac.core.scenes.scene import Scene
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdPhysics, Usd, Sdf
from src.config import Config


class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "denso_pick_place",
        cfg: Config = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cfg.target_position,
            cube_initial_orientation=cfg.target_orientation,
            target_position=cfg.husky_target_pose,
            cube_size=np.array(cfg.target_cube_size),
            offset=cfg.target_cube_offset,
        )
        self.robots = {}
        self.cfg = cfg
        self.husky_asset_path = cfg.husky_usd_path
        return

    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        print("\n Now we are in new set_up_scene!!! \n")

        husky_asset_path = self.husky_asset_path #"/home/vitaly/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/ur5_gripper_husky_obj/asset/husky.usd"
        self.husky = scene.add(
        WheeledRobot(
            prim_path=self.cfg.husky_stage_path,
            name="robot",
            wheel_dof_names=["rear_left_wheel_joint", "rear_right_wheel_joint", "front_left_wheel_joint", "front_right_wheel_joint"],
            create_robot=True,
            usd_path=husky_asset_path,
            position=np.array(self.cfg.husky_init_pose),
            # orientation=np.array([1, 0, 0, 0.]),
            )
        )
        self.robots["husky"] = self.husky
        print(f"husky: {self.husky}")
        #########################################################################
        # Disable掉固定UR5的root_joint  --> 设置Husky 和 UR5 之间的连接关节
        stage = get_current_stage()
        prim_root_joint = stage.GetPrimAtPath(f"{self.cfg.ur5_stage_path}/root_joint")
        # print(f"\n prim_root_joint: {prim_root_joint}\n")
        root_joint = UsdPhysics.Joint(prim_root_joint)
        root_joint_enable = root_joint.GetJointEnabledAttr().Get()
        # print(f"\nroot_joint_enable: {root_joint_enable}\n")
        root_joint_enable = root_joint.GetJointEnabledAttr().Set(False)
        # print(f"\nroot_joint_enable: {root_joint_enable}\n")

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, self.cfg.connection_joint_stage_path)
        # print(f"fixed_joint: {fixed_joint}\n")

        fixed_joint.GetBody0Rel().SetTargets([f"{self.cfg.husky_stage_path}/put_ur5"])  #[0.3312 0 0.25178]
        fixed_joint.GetBody1Rel().SetTargets([f"{self.cfg.ur5_stage_path}/world"])
        fixed_joint.GetExcludeFromArticulationAttr().Set(True)
        #########################################################################	
        return

    def set_robot(self) -> SingleManipulator:
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")

        asset_path = self.cfg.ur5_usd_path # 确定可以

        # 配置好的机械爪约束
        # asset_path = "/home/zhang/save/8-2/new3/new3.usd"

        add_reference_to_stage(usd_path=os.path.abspath(asset_path), prim_path=self.cfg.ur5_stage_path)

        # finger_joint    right_outer_kunckle_joint
        # left_inner_finger_joint     right_inner_finger_joint
        # left_inner_knuckle_joint     right_inner_knuckle_joint
        gripper = ParallelGripper( 
            end_effector_prim_path=self.cfg.end_effector_stage_path,
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.4, -0.4]),
            # action_deltas=np.array([-0.05, 0.05]),
        )
        
        manipulator = SingleManipulator(
            prim_path=self.cfg.ur5_stage_path,
            name="my_ur5",
            end_effector_prim_name="ur5_ee_link",
            gripper=gripper,
            translation = np.array([0.3312, 0, 0.407]),  
            # orientation = np.array([1, 0, 0, 0.]),
        )
        self.robots["ur5"] = manipulator
        joints_default_positions = np.zeros(12)
        # joints_default_positions[0] = 0
        joints_default_positions[1] = -np.pi / 2
        # joints_default_positions[2] = 0
        # joints_default_positions[3] = np.pi / 2
        joints_default_positions[4] = np.pi / 2
        manipulator.set_joints_default_state(positions=joints_default_positions)

        return manipulator

    # def get_observations(self) -> dict:
    #     """[summary]

    #     Returns:
    #         dict: [description]
    #     """
    #     # print("\n Now listen to me ! \n")
    #     joints_state = self._robot.get_joints_state()
    #     target_position, target_orientation = self._target.get_local_pose()
    #     return {
    #         self._robot.name: {
    #             "joint_positions": np.array(joints_state.positions),
    #             "joint_velocities": np.array(joints_state.velocities),
    #         },
    #         self._target.name: {"position": np.array(target_position), "orientation": np.array(target_orientation)},
    #     }
    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        cube_position, cube_orientation = self._cube.get_local_pose()
        end_effector_position, _ = self._robot.end_effector.get_local_pose()
        return {
            self._cube.name: {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": self._target_position,
            },
            self._robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
            },
        }
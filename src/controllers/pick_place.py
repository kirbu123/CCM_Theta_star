# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.isaac.manipulators.controllers as manipulators_controllers
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.articulations import Articulation
from src.config import Config
from .rmpflow import RMPFlowController

class PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(self,
                 name: str,
                 gripper: ParallelGripper,
                 robot_articulation: Articulation,
                 cfg: Config=None) -> None:
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            events_dt=cfg.control.pick_place.events_dt,
            end_effector_initial_height=cfg.control.pick_place.end_effector_initial_height,
        )
        return

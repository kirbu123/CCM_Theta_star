from typing import Dict, List, Union

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.materials import PreviewSurface
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name

from src.config import get_config

cfg = get_config()


def add_goals(world: World, scene: Dict, goals: List):
    """
    Add visualizations of goals as small red cubes to the scene.

    Args:
        world (World): The simulation world.
        scene (Scene): The simulation scene.
        goals (List[Tuple[float, float]]): A list of (x, y) coordinates of the goals.

    Returns:
        None
    """
    for i, (x, y) in enumerate(goals):
        color = np.array([1.0, 0, 0])
        goal = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/goal_cube_{i}",  # The prim path of the cube in the USD stage
                name=f"goal_cube_{i}",  # The unique name used to retrieve the object from the scene later on
                position=np.array(
                    [x, y, 0.1]
                ),  # Using the current stage units which is in meters by default.
                scale=np.array([0.07, 0.07, 0.07]),  # most arguments accept mainly numpy arrays.
                color=color,  # RGB channels, going from 0-1
            )
        )

        scene[f"goal_{i}"] = goal

    complete_goal_prim_path = find_unique_string_name(
        initial_name="/World/Looks/complete_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
    )
    scene["complete_goal_material"] = PreviewSurface(
        prim_path=complete_goal_prim_path, color=np.array([0.0, 1.0, 0])
    )

    complete_goal_prim_path = find_unique_string_name(
        initial_name="/World/Looks/next_goal_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
    )
    scene["next_goal_material"] = PreviewSurface(
        prim_path=complete_goal_prim_path, color=np.array([1.0, 1.0, 0])
    )

    world.reset()


def add_cube(
    world: World,
    name: str,
    position: Union[List, np.ndarray] = np.zeros(3),
    scale: Union[List, np.ndarray] = np.ones(3),
    color: Union[List, np.ndarray, None] = None,
) -> None:
    """
    Add a cube to the scene.

    Args:
        world (World): The simulation world.
        name (str): The name of the cube on the scene.
        position (np.ndarray): The position of the cube.
        scale (np.ndarray): The scale of the cube.
        color (np.ndarray): The color of the cube.

    Returns:
        None
    """

    position = np.array(position)
    scale = np.array(scale)
    if color is not None:
        color = np.array(color)
    else:
        color = np.array([1.0, 1.0, 1.0]) / 3.0

    world.scene.add(
        DynamicCuboid(
            prim_path=f"{cfg.background_stage_path}/{name}",  # The prim path of the cube in the USD stage
            name=name,  # The unique name used to retrieve the object from the scene later on
            position=position,  # Using the current stage units which is in meters by default.
            scale=scale,  # most arguments accept mainly numpy arrays.
            color=color,  # RGB channels, going from 0-1
        )
    )

    world.reset()

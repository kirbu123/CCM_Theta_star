from pathlib import Path
from queue import Queue
from typing import Dict, Tuple, Union

import yaml
from omni.isaac.core import World

from src.config import Config, get_config
from src.scene_setup.objects import add_cube

cfg = get_config()

if cfg.mode == "online":
    if cfg.ros == "noetic":
        from communication_msgs.msg import MoveToGoal, PickupObjectGoal, PutObjectGoal
    else:
        # Here we import ROS2 packages
        # TODO
        raise NotImplementedError("ROS2  Actions is not implemented yet")
        pass
else:
    from src.actions.communication_msgs import (
        MoveToGoal,
        PickupObjectGoal,
        PutObjectGoal,
    )


def _task_to_goal(task: Dict) -> Tuple[str, Union[MoveToGoal, PickupObjectGoal, PutObjectGoal]]:
    task_type = task["type"]
    goal = None
    server_name = None

    if task_type == "MOVE_TO":
        goal = MoveToGoal(
            location=task["location"],
            object=task["object"],
        )
        server_name = "move_to_location"
    elif task_type == "PICK_UP":
        goal = PickupObjectGoal(
            location=task["location"],
            object=task["object"],
        )
        server_name = "pick_up_object"
    elif task_type == "PUT":
        goal = PutObjectGoal(
            location=task["location"],
            object=task["object"],
        )
        server_name = "put_object"
    else:
        raise ValueError(f"Unknown task type: {task_type}")

    return (server_name, goal)


def add_scenario_to_queue(cfg: Config, queue: Queue) -> None:
    path = Path(f"{cfg.scenarios_dir}/{cfg.scenario_name}.yaml")
    if not path.is_file():
        raise ValueError("Invalid file path or extension")

    with open(str(path), "r") as file:
        try:
            data = yaml.safe_load(file)
            scenarios = data.get("scenario", [])
            for task in scenarios:
                goal = _task_to_goal(task)
                queue.put(goal)
        except yaml.YAMLError as e:
            print(f"Error reading YAML file: {e}")


def setup_scene_for_scenario(cfg: Config, world: World) -> None:
    path = Path(f"{cfg.scenarios_dir}/{cfg.scenario_name}.yaml")
    if not path.is_file():
        raise ValueError("Invalid file path or extension")

    with open(str(path), "r") as file:
        try:
            data = yaml.safe_load(file)
            env_cfg = data.get("environment", {})
            if env_cfg["fixed"]:
                print("Environment is fixed, nothing to do.")
                return

            objects = env_cfg["objects"]
            for object in objects:
                if object["type"] == "cube":
                    if not object["has_texture"]:
                        color = object["color"]
                    else:
                        color = None
                    add_cube(
                        world=world,
                        name=object["name"],
                        position=object["position"],
                        scale=object["scale"],
                        color=color,
                    )
                else:
                    raise NotImplementedError(f"Object type: {object['type']} is not supported for spawn yet")

        except yaml.YAMLError as e:
            print(f"Error reading YAML file: {e}")

from pathlib import Path
from queue import Queue
from typing import Dict, Tuple, Union

import yaml
from communication_msgs.msg import MoveToGoal, PickupObjectGoal, PutObjectGoal

from src.config import Config


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
    path = Path(f"{cfg.scenarios_dir}/{cfg.scenaio_name}.yaml")
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

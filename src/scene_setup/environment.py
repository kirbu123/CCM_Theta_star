import sys

import carb
from omni.isaac.core.utils import nucleus, stage
from omni.isaac.kit import SimulationApp

from src.config import Config


def setup_scene_background(simulation_app: SimulationApp, cfg: Config):
    # Locate Isaac Sim assets folder to load environment and robot stages
    assets_root_path = nucleus.get_assets_root_path()  # !
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()

    # Loading the simple_room environment
    if cfg.use_background:
        if cfg.use_omni_background:
            stage.add_reference_to_stage(
                assets_root_path + cfg.background_usd_path, cfg.background_stage_path
            )  # !
        else:
            stage.add_reference_to_stage(cfg.background_usd_path, cfg.background_stage_path)
    else:
        stage.add_reference_to_stage(
            assets_root_path + "/Isaac/Environments/Grid/default_environment.usd", cfg.background_stage_path
        )

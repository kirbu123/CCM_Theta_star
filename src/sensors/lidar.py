from pxr.Usd import Stage
import omni.graph.core as og
from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.prims import set_targets

from src.config import Config


def setup_lidar_graph(cfg: Config, simulation_app: SimulationApp, stage: Stage):
    '''Setup the LiDAR action graph for publishing the point cloud to ROS2'''
    try:
        og.Controller.edit(
            {"graph_path": cfg.lidar.action_graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),

                    ("createLiRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("lidarHelperMsg", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ("lidarHelperPointcloud", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createLiRenderProduct.inputs:execIn"),
                    ("createLiRenderProduct.outputs:execOut", "lidarHelperMsg.inputs:execIn"),
                    ("createLiRenderProduct.outputs:execOut", "lidarHelperPointcloud.inputs:execIn"),
                    ("createLiRenderProduct.outputs:renderProductPath", "lidarHelperMsg.inputs:renderProductPath"),
                    ("createLiRenderProduct.outputs:renderProductPath", "lidarHelperPointcloud.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ##? LiDAR
                    ("lidarHelperPointcloud.inputs:topicName", "lidar"),
                    ("lidarHelperPointcloud.inputs:frameId", "rtx_lidar"),
                    ("lidarHelperPointcloud.inputs:nodeNamespace", "/sensor"),
                    ("lidarHelperPointcloud.inputs:type", "point_cloud"),
                ],
            },
        )
    except Exception as e:
        print(e)

    set_targets(
        prim=stage.GetPrimAtPath(cfg.lidar.action_graph_path + "/createLiRenderProduct"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg.lidar.lidar_stage_path],
    )

    simulation_app.update()
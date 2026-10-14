import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Run the active Go2 controller stack in the TI-building world."""
    sim_share = get_package_share_directory("unitree_go2_sim")
    description_share = get_package_share_directory("unitree_go2_description")

    standard_launch = os.path.join(
        sim_share, "launch", "unitree_go2_launch.py"
    )
    ti_world = os.path.join(
        description_share, "worlds", "TIbuilding.sdf"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(standard_launch),
                launch_arguments={
                    "world": ti_world,
                    "world_init_x": "0.0",
                    "world_init_y": "0.0",
                    "world_init_z": "4.375",
                    "world_init_heading": "0.0",
                }.items(),
            )
        ]
    )

"""华汇悟时：零售导航门面 + Agent HTTP 网关（默认 adapter:=woosh）."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("retail_nav_bridge")
    default_params = os.path.join(pkg_share, "config", "nav_defaults_woosh.yaml")
    default_stations = os.path.join(pkg_share, "config", "retail_stations.yaml")
    default_target_mapping = os.path.join(
        pkg_share, "config", "product_slot_navigation.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("stations_file", default_value=default_stations),
            DeclareLaunchArgument(
                "target_mapping_file", default_value=default_target_mapping
            ),
            DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("http_port", default_value="8081"),
            Node(
                package="retail_nav_bridge",
                executable="retail_nav_bridge",
                name="retail_nav_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "adapter": "woosh",
                        "stations_file": LaunchConfiguration("stations_file"),
                    },
                ],
            ),
            Node(
                package="retail_nav_bridge",
                executable="retail_nav_http_gateway",
                name="retail_nav_http_gateway",
                output="screen",
                parameters=[
                    {
                        "http_host": LaunchConfiguration("http_host"),
                        "http_port": ParameterValue(
                            LaunchConfiguration("http_port"), value_type=int
                        ),
                        "stations_file": LaunchConfiguration("stations_file"),
                        "target_mapping_file": LaunchConfiguration(
                            "target_mapping_file"
                        ),
                        "default_max_speed": 0.15,
                    }
                ],
            ),
        ]
    )

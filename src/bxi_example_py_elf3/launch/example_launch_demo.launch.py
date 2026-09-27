import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json

def generate_launch_description():

    xml_file_name = "data/elf3.xml"
    xml_file = os.path.join(get_package_share_path("bxi_example_py_elf3"), xml_file_name)
    remote_share = get_package_share_path("remote_controller")

    npz_file_dict = {
        "dance": "data/dance.npz",
    }  
    onnx_file_dict = {
        "normal": "data/model_normal.onnx",
        "host": "data/host.onnx",
        "dance": "data/dance.onnx",
    }

    for key, value in npz_file_dict.items():
        npz_file_dict[key] = os.path.join(get_package_share_path("bxi_example_py_elf3"), value)
    for key, value in onnx_file_dict.items():
        onnx_file_dict[key] = os.path.join(get_package_share_path("bxi_example_py_elf3"), value)

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_remote_controller", default_value="false"),
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),

            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/release_suspension": True},
                    {"/npz_file_dict": json.dumps(npz_file_dict)},
                    {"/onnx_file_dict": json.dumps(onnx_file_dict)},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
            Node(
                package="remote_controller",
                executable="remote_controller",
                name="remote_controller_sim",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_remote_controller")),
                arguments=[
                    "--config",
                    str(remote_share / "config/xbox_default.yaml"),
                    "--disable-system-actions",
                ],
            ),
        ]
    )

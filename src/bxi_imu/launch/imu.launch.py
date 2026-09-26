# Copyright 2026 BXI Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import glob

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("bxi_imu")
    module_root = os.path.join(package_share, "modules")
    module_configs = sorted(glob.glob(os.path.join(module_root, "*", "config.yaml")))

    candidates = []
    for config_path in module_configs:
        with open(config_path, "r", encoding="utf-8") as config_file:
            document = yaml.safe_load(config_file) or {}
        parameters = document.get("imu_module", {}).get("ros__parameters", {})
        driver = str(parameters.get("driver", "")).strip()
        port = str(parameters.get("port", "")).strip()
        baudrate = int(parameters.get("baudrate", 921600))
        if driver and port:
            axis_mapping = str(parameters.get("axis_mapping", "identity"))
            frequency = float(parameters.get("imu_frequency_hz", 200.0))
            timeout_multiplier = float(parameters.get("imu_timeout_multiplier", 1.5))
            candidates.append(
                "|".join(
                    [
                        driver,
                        port,
                        str(baudrate),
                        axis_mapping,
                        str(frequency),
                        str(timeout_multiplier),
                        str(parameters.get("frame_id", "imu_link")),
                        str(parameters.get("imu_topic", "/hardware/imu_data")),
                        str(parameters.get("euler_topic", "/euler_data")),
                        str(parameters.get("magnetic_topic", "/magnetic_data")),
                        str(parameters.get("temperature_topic", "/temp_data")),
                        str(parameters.get("pressure_topic", "/pressure_data")),
                        str(parameters.get("imu_enabled", True)).lower(),
                        str(parameters.get("quaternion_norm_tolerance", 0.1)),
                        str(parameters.get("euler_enabled", False)).lower(),
                        str(parameters.get("magnetic_enabled", False)).lower(),
                        str(parameters.get("temperature_enabled", False)).lower(),
                        str(parameters.get("pressure_enabled", False)).lower(),
                        str(parameters.get("imu_record_enabled", False)).lower(),
                        str(parameters.get("imu_record_dir", "/var/log/bxi_log/imu/data")),
                        str(parameters.get("imu_record_max_files", 10)),
                    ]
                )
            )

    if not candidates:
        raise RuntimeError(f"No IMU module config found under {module_root}")

    return LaunchDescription(
        [
            DeclareLaunchArgument("driver", default_value="auto"),
            DeclareLaunchArgument("port", default_value="auto"),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument(
                "imu_record_enabled", default_value="auto",
                description="Override module CSV recording setting (auto/true/false)",
            ),
            Node(
                package="bxi_imu",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[
                    {
                        "driver": LaunchConfiguration("driver"),
                        "port": LaunchConfiguration("port"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"), value_type=int
                        ),
                        "imu_candidates": candidates,
                        "imu_record_enabled_override": ParameterValue(
                            LaunchConfiguration("imu_record_enabled"), value_type=str
                        ),
                    },
                ],
            ),
        ]
    )

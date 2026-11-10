from pathlib import Path
from xml.etree import ElementTree

from launch import LaunchDescription
from launch_ros.actions import Node


MODEL = Path(__file__).resolve().parents[1] / "urdf/elf3_gripper.urdf"
RVIZ_CONFIG = Path(__file__).resolve().parents[1] / "rviz/display.rviz"


def generate_launch_description():
    robot = ElementTree.parse(MODEL).getroot()
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename")
        if not filename or "://" in filename:
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = MODEL.parent / path
        mesh.set("filename", path.resolve(strict=True).as_uri())

    description = ElementTree.tostring(robot, encoding="unicode")
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"use_mimic_tags": True}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(RVIZ_CONFIG)],
                output="screen",
            ),
        ]
    )

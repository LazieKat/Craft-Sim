"""Single launch file for the full spacecraft attitude control pipeline.

Default parameters are tuned for a stable step response with the default
inertia (Ixx=1, Iyy=2, Izz=3 kg·m²).  All values can be overridden on the
command line, e.g.:
    ros2 launch craft_sim craft_sim.launch.py att_kp:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    args = [
        # Inertia
        declare("ixx", "1.0",  "Moment of inertia Ixx (kg·m²)"),
        declare("iyy", "2.0",  "Moment of inertia Iyy (kg·m²)"),
        declare("izz", "3.0",  "Moment of inertia Izz (kg·m²)"),
        # Attitude PID
        declare("att_kp", "2.0",  "Attitude controller proportional gain"),
        declare("att_ki", "0.05", "Attitude controller integral gain"),
        declare("att_kd", "0.5",  "Attitude controller derivative gain"),
        declare("att_ff", "0.0",  "Attitude controller feed-forward gain"),
        # Rate PD
        declare("rate_kp", "10.0", "Rate controller proportional gain"),
        declare("rate_kd", "0.5",  "Rate controller derivative gain"),
        declare("rate_ff", "0.0",  "Rate controller feed-forward gain"),
    ]

    # Convenience: build LaunchConfiguration references once
    cfg = {name: LaunchConfiguration(name) for name in [
        "ixx", "iyy", "izz",
        "att_kp", "att_ki", "att_kd", "att_ff",
        "rate_kp", "rate_kd", "rate_ff",
    ]}

    nodes = [
        Node(
            package="craft_sim",
            executable="simulator_node",
            name="simulator_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="craft_sim",
            executable="rate_controller_node",
            name="rate_controller_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="craft_sim",
            executable="attitude_controller_node",
            name="attitude_controller_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="craft_sim",
            executable="flight_controller_node.py",
            name="flight_controller_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "ixx":     cfg["ixx"],
                "iyy":     cfg["iyy"],
                "izz":     cfg["izz"],
                "att_kp":  cfg["att_kp"],
                "att_ki":  cfg["att_ki"],
                "att_kd":  cfg["att_kd"],
                "att_ff":  cfg["att_ff"],
                "rate_kp": cfg["rate_kp"],
                "rate_kd": cfg["rate_kd"],
                "rate_ff": cfg["rate_ff"],
            }],
        ),
        Node(
            package="craft_sim",
            executable="visualizer_node.py",
            name="visualizer_node",
            output="screen",
            emulate_tty=True,
        ),
    ]

    return LaunchDescription(args + nodes)

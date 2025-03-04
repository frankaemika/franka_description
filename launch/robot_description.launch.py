from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity, Substitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parse_robot_description(
    context: LaunchContext,
    robot_ip: Substitution,
    arm_id: Substitution,
    use_fake_hardware: Substitution,
    fake_sensor_commands: Substitution,
    load_gripper: Substitution,
) -> list[LaunchDescriptionEntity]:
    """Generate the robot description using xacro and start the robot."""
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = str(
        Path(get_package_share_directory("franka_description")) / "robots" / arm_id_str / (arm_id_str + ".urdf.xacro")
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "ros2_control": "true",
            "arm_id": arm_id_str,
            "robot_ip": robot_ip_str,
            "hand": load_gripper_str,
            "use_fake_hardware": use_fake_hardware_str,
            "fake_sensor_commands": fake_sensor_commands_str,
        },
    ).toprettyxml(indent="  ")  # type: ignore[reportAttributeAccessIssue]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Parse the robot model URDF file and start the robot state publisher."""
    # ---------- Declare launch arguments ----------

    launch_args = []

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="Hostname or IP address of the robot.",
    )
    launch_args.append(robot_ip_arg)

    # Redeclare the arm ID argument because we want to add a default option.
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        description="ID of the type of arm used.",
        choices=["fer", "fr3", "fp3"],
    )
    launch_args.append(arm_id_arg)

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        choices=["true", "false"],
        description="Use fake hardware",
    )

    launch_args.append(use_fake_hardware_arg)

    fake_sensor_commands_arg = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
        choices=["true", "false"],
        description=f'Fake sensor commands. Only valid when "{use_fake_hardware_arg.name}" is true',
    )
    launch_args.append(fake_sensor_commands_arg)

    load_gripper_arg = DeclareLaunchArgument(
        "load_gripper",
        default_value="true",
        choices=["true", "false"],
        description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.",
    )
    launch_args.append(load_gripper_arg)

    # ---------- Declare executables ----------

    # Generate the URDF model and use it to start the robot_state_publisher
    robot_description_and_state_publisher = OpaqueFunction(
        function=parse_robot_description,
        args=[
            LaunchConfiguration(robot_ip_arg.name),
            LaunchConfiguration(arm_id_arg.name),
            LaunchConfiguration(use_fake_hardware_arg.name),
            LaunchConfiguration(fake_sensor_commands_arg.name),
            LaunchConfiguration(load_gripper_arg.name),
        ],
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            robot_description_and_state_publisher,
        ]
    )

#  Copyright (c) 2023 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import argparse
import xacro


def str_to_bool(s):
    return s.lower() in ["true", "1", "yes", "y"]


def convert_xacro_to_urdf(xacro_file, with_sc, hand):
    """Convert xacro file into a URDF file."""
    urdf = xacro.process_file(
        xacro_file, mappings={"with_sc": str(with_sc), "hand": str(hand)}
    )
    urdf_file = urdf.toprettyxml(indent="  ")
    return urdf_file


def convert_package_name_to_absolute_path(package_name, package_path, urdf_file):
    """Replace a ROS package names with the absolute paths."""
    urdf_file = urdf_file.replace("package://{}".format(package_name), package_path)
    return urdf_file


def save_urdf_to_file(package_path, urdf_file, robot):
    """Save URDF into a file."""
    # Check if the folder exists, and if not, create it
    folder_path = f"{package_path}/urdfs"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    with open(f"{package_path}/urdfs/{robot}.urdf", "w") as f:
        f.write(urdf_file)


if __name__ == "__main__":
    package_name = "franka_description"

    if os.getcwd().split("/")[-1] != package_name:
        print("Call the script from franka_description root folder")
        exit()

    ROBOTS = ["fr3", "fp3", "fer"]

    parser = argparse.ArgumentParser(
        description="""
            Generate franka robots urdf models.
            Script to be executed from franka_description root folder!
            """
    )

    robots_str = ", ".join(["{}" for item in ROBOTS]).format(*ROBOTS)
    parser.add_argument(
        "robot_model",
        type=str,
        help="id of the robot model (accepted values are: {})".format(robots_str),
    )
    parser.add_argument(
        "--no-hand",
        help="Disable loading of franka hand.",
        action="store_const",
        const=True,
    )
    parser.add_argument(
        "--with-sc",
        help="Include self-collision volumes in the urdf model.",
        action="store_const",
        const=True,
    )
    parser.add_argument(
        "--abs-path", help="Use absolute paths.", action="store_const", const=True
    )
    parser.add_argument(
        "--host-dir", help="Provide a host directory for the absolute path."
    )

    args = parser.parse_args()

    ROBOT_MODEL = args.robot_model.lower()
    HAND = not args.no_hand
    WITH_SC = args.with_sc if args.with_sc is not None else False
    ABSOLUTE_PATHS = args.abs_path if args.abs_path is not None else False
    HOST_DIR = args.host_dir

    assert ROBOT_MODEL in ROBOTS or ROBOT_MODEL == "all"

    if ROBOT_MODEL != "all":
        ROBOTS = [ROBOT_MODEL]

    for robot in ROBOTS:
        print(f"\n*** Creating URDF for {robot} ***")
        xacro_file = f"robots/{robot}/{robot}.urdf.xacro"

        package_path = os.getcwd()

        xacro_file = os.path.join(package_path, xacro_file)

        urdf_file = convert_xacro_to_urdf(xacro_file, WITH_SC, HAND)

        if ABSOLUTE_PATHS and (HOST_DIR is None or HOST_DIR == ""):
            urdf_file = convert_package_name_to_absolute_path(
                package_name, package_path, urdf_file
            )
        elif ABSOLUTE_PATHS:
            urdf_file = convert_package_name_to_absolute_path(
                package_name, HOST_DIR, urdf_file
            )

        save_urdf_to_file(package_path, urdf_file, robot)

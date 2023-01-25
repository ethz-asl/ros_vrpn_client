# Copyright 2023 ASL

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

import yaml

object_names = ["object_1", "object_2", "object_3", "object_4"]


def get_params(package, file, name):
    path = os.path.join(get_package_share_directory(package) + file)

    with open(path, "r") as file:
        params = yaml.safe_load(file)[name]["ros__parameters"]
    return params


def generate_launch_description():

    params = get_params(
        "ros_vrpn", "/config/asl_vicon.yaml", "ros_vrpn_client"
    )

    nodes = []
    for object in object_names:
        params["object_name"] = object
        nodes.append(
            Node(
                package="ros_vrpn",
                executable="ros_vrpn_client",
                namespace=object,
                output="screen",
                parameters=[params],
            )
        )

    return LaunchDescription(nodes)

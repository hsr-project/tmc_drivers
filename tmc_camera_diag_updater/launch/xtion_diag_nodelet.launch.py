#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('rgb_image_topic_name',
                              default_value='rgb_image',
                              description='Subscription rgb topic name'))
    declared_arguments.append(
        DeclareLaunchArgument('depth_image_topic_name',
                              default_value='depth_image',
                              description='Subscription depth topic name'))
    declared_arguments.append(
        DeclareLaunchArgument('points_topic_name',
                              default_value='points',
                              description='Subscription points topic name'))
    declared_arguments.append(
        DeclareLaunchArgument('use_color_image',
                              default_value='true',
                              description='Use the color diagnostics'))
    declared_arguments.append(
        DeclareLaunchArgument('use_depth_image',
                              default_value='true',
                              description='Use the depth diagnostics'))
    declared_arguments.append(
        DeclareLaunchArgument('use_point_cloud',
                              default_value='true',
                              description='Use the point diagnostics'))
    return declared_arguments


def generate_launch_description():
    color_camera_diag_component = ComposableNodeContainer(
        name="color_img_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="color_img_diag_updater",
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': LaunchConfiguration('rgb_image_topic_name'),
                             'topic_type': 'image',
                             'warning_hz': 5.0}],
                condition=IfCondition(LaunchConfiguration('use_color_image')))
        ]
    )

    depth_camera_diag_component = ComposableNodeContainer(
        name="depth_img_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="depth_img_diag_updater",
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': LaunchConfiguration('depth_image_topic_name'),
                             'topic_type': 'image',
                             'warning_hz': 5.0,
                             'unicolor_check': False}],
                condition=IfCondition(LaunchConfiguration('use_depth_image')))
        ]
    )

    points_camera_diag_component = ComposableNodeContainer(
        name="points_img_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="points_diag_updater",
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': LaunchConfiguration('points_topic_name'),
                             'topic_type': 'points',
                             'warning_hz': 2.0}],
                condition=IfCondition(LaunchConfiguration('use_point_cloud')))
        ]
    )

    return LaunchDescription(declare_arguments() + [color_camera_diag_component, depth_camera_diag_component,
                                                    points_camera_diag_component])

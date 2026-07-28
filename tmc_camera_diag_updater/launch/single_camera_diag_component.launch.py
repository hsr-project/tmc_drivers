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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('node_name',
                              default_value='diag_updater',
                              description='Name of the launch node'))
    declared_arguments.append(
        DeclareLaunchArgument('topic_name',
                              default_value='image',
                              description='Subscription topic name'))
    declared_arguments.append(
        DeclareLaunchArgument('warning_hz',
                              default_value='5.0',
                              description='Definition of topic subscription cycle warn Level'))
    declared_arguments.append(
        DeclareLaunchArgument('hardware_id',
                              default_value='hardware_id',
                              description='Hardware ID of the device issuing the diagnostic report'))
    return declared_arguments


def generate_launch_description():
    camera_diag_component = ComposableNodeContainer(
        name="single_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name=LaunchConfiguration('node_name'),
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': LaunchConfiguration('topic_name'),
                             'topic_type': 'image',
                             'warning_hz': LaunchConfiguration('warning_hz'),
                             'hardware_id': LaunchConfiguration('hardware_id')}])
        ]
    )

    return LaunchDescription(declare_arguments() + [camera_diag_component])

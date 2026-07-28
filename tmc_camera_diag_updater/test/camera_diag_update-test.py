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
import launch
from launch import (
    LaunchDescription,
    LaunchService
)
from launch.events import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.actions import GTest


def test_diag_updater_node_test():
    def check_test_result(event, context):
        assert 0 == event.returncode
        return launch.actions.EmitEvent(event=Shutdown())

    camera_diag_component = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                name="color_img_diag_updater",
                namespace='camera',
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': "rgb/image_rect_color",
                             'topic_type': 'image',
                             'warning_hz': 20.0,
                             'sub_timeout': 1.0,
                             'diag_pub_rate': 10.0}]),
            ComposableNode(
                name="points_diag_updater",
                namespace='camera',
                package="tmc_camera_diag_updater",
                plugin="camera_diag_updater::CameraDiagComponent",
                parameters=[{'topic_name': "depth_registered/points",
                             'topic_type': 'points',
                             'warning_hz': 10.0,
                             'sub_timeout': 1.0,
                             'diag_pub_rate': 10.0}]),
        ]
    )

    camera_node_test = GTest(
        path='camera_diag_update_test',
        timeout=120.0,
        on_exit=check_test_result,
        output='screen')

    """Launch a gtest."""
    ld = LaunchDescription([
        camera_diag_component,
        camera_node_test
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

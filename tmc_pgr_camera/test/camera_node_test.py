# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
from ament_index_python.packages import get_package_share_directory
import launch
from launch import (
    LaunchDescription,
    LaunchService
)
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import GTest


def test_diag_updater_node_test():
    config_path = get_package_share_directory('tmc_pgr_camera')

    def check_test_result(event, context):
        assert 0 == event.returncode
        return launch.actions.EmitEvent(event=Shutdown())

    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [config_path + '/launch',
             '/capture.launch.py']),
        launch_arguments={
            'node_name': 'stereo_camera',
            'plugin_name': 'tmc_pgr_camera/dummy_camera_system_plugin',
            'frame_id': 'pgr_stereo_camera',
            'camera_setting_file_path': 'test/config/default.yml',
            'image_topic_names': "['/stereo_camera/left/image_raw', '/stereo_camera/right/image_raw']",
        }.items())

    camera_node_test = GTest(
        path='camera_node-test',
        timeout=60.0,
        on_exit=check_test_result,
        output='screen')

    """Launch a gtest."""
    ld = LaunchDescription([
        capture,
        camera_node_test
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

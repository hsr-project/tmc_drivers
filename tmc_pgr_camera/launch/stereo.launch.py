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
from distutils.util import strtobool
import os

from launch import (
    LaunchDescription
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
)


def launch_setup(context, *args, **kwargs):
    use_blackfly = strtobool(context.perform_substitution(LaunchConfiguration('use_blackfly')))

    launch_args = {
        'node_name': 'stereo_camera',
        'frame_id': LaunchConfiguration('frame_id'),
        'camera_setting_file_path': LaunchConfiguration('camera_setting_file_path'),
        'image_topic_names': LaunchConfiguration('image_topic_names'),
    }
    if use_blackfly:
        launch_args['self_trigger/number_of_pulse'] = '255'
        launch_args['self_trigger/polarity'] = '1'
        launch_args['self_trigger/pulse_width'] = '[63, 63]'

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/capture.launch.py']),
        launch_arguments=launch_args.items())]


def generate_launch_description():
    ros_home_dir = os.environ.get('ROS_HOME', '~/.ros')

    args = [
        DeclareLaunchArgument('frame_id', default_value='pgr_stereo_camera'),
        DeclareLaunchArgument('camera_setting_file_path',
                              default_value=ros_home_dir + '/tmc/robot/conf.d/stereo_pgr_camera.yml'),
        DeclareLaunchArgument('image_topic_names',
                              default_value='["/stereo_camera/left/image_raw", "/stereo_camera/right/image_raw"]'),
        DeclareLaunchArgument('use_blackfly', default_value='False'),
    ]

    ld = LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
    return ld

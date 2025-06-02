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
from launch import (
    LaunchDescription
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Fundamental
    args_fundamentals = [
        DeclareLaunchArgument('node_name', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('auto_capture_start', default_value='True'),
        DeclareLaunchArgument(
            'plugin_export_tag_name', default_value='tmc_pgr_camera'),
        DeclareLaunchArgument(
            'plugin_name', default_value='tmc_pgr_camera/point_grey_camera_system'),
        DeclareLaunchArgument('camera_setting_file_path', default_value=''),
        DeclareLaunchArgument('image_topic_names', default_value='["", ""]'),
    ]

    # Properties
    args_properties = [
        DeclareLaunchArgument(
            'property/brightness/on_off', default_value='True'),
        DeclareLaunchArgument(
            'property/brightness/abs_value', default_value='0.0'),
        DeclareLaunchArgument(
            'property/brightness/one_push', default_value='True'),
        DeclareLaunchArgument(
            'property/brightness/auto_manual_mode', default_value='False'),
        DeclareLaunchArgument(
            'property/auto_exposure/on_off', default_value='True'),
        DeclareLaunchArgument(
            'property/auto_exposure/abs_value', default_value='0.0'),
        DeclareLaunchArgument(
            'property/auto_exposure/one_push', default_value='True'),
        DeclareLaunchArgument(
            'property/auto_exposure/auto_manual_mode', default_value='True'),
        DeclareLaunchArgument(
            'property/white_balance/on_off', default_value='True'),
        DeclareLaunchArgument(
            'property/white_balance/value_a', default_value='570'),
        DeclareLaunchArgument(
            'property/white_balance/value_b', default_value='810'),
        DeclareLaunchArgument(
            'property/white_balance/one_push', default_value='True'),
        DeclareLaunchArgument(
            'property/white_balance/auto_manual_mode', default_value='True'),
        DeclareLaunchArgument(
            'property/shutter/on_off', default_value='True'),
        DeclareLaunchArgument(
            'property/shutter/abs_value', default_value='20.0'),
        DeclareLaunchArgument(
            'property/shutter/one_push', default_value='False'),
        DeclareLaunchArgument(
            'property/shutter/auto_manual_mode', default_value='False'),
        DeclareLaunchArgument(
            'property/gain/on_off', default_value='True'),
        DeclareLaunchArgument(
            'property/gain/abs_value', default_value='0.0'),
        DeclareLaunchArgument(
            'property/gain/one_push', default_value='False'),
        DeclareLaunchArgument(
            'property/gain/auto_manual_mode', default_value='False'),
        DeclareLaunchArgument(
            'property/trigger_delay/on_off', default_value='False'),
        DeclareLaunchArgument(
            'property/trigger_delay/abs_value', default_value='5.0'),
        DeclareLaunchArgument(
            'property/trigger_delay/abs_control', default_value='True'),
    ]

    # Format7
    args_format7 = [
        DeclareLaunchArgument('format7/mode', default_value='0'),
        DeclareLaunchArgument('format7/offset_x', default_value='8'),
        DeclareLaunchArgument('format7/offset_y', default_value='2'),
        DeclareLaunchArgument('format7/width', default_value='1280'),
        DeclareLaunchArgument('format7/height', default_value='960'),
        DeclareLaunchArgument('format7/pixel_format', default_value='raw8'),
    ]

    # Sync pulse setting
    args_sync_pulse_setting = [
        # Value mean [GPIO_IN, GPIO_OUT]
        DeclareLaunchArgument('self_trigger/io', default_value='[0, 1]'),
        # Value mean [HIGH, LOW] (msec)
        DeclareLaunchArgument('self_trigger/pulse_width', default_value='[1, 16]'),
        DeclareLaunchArgument('self_trigger/number_of_pulse', default_value='1'),
        DeclareLaunchArgument('self_trigger/polarity', default_value='0'),
    ]

    # Other
    args_other = [
        DeclareLaunchArgument('software_demosaicing', default_value='edge_sensing'),
        DeclareLaunchArgument('video_mode', default_value='format7'),
        DeclareLaunchArgument('frame_rate', default_value='5.0'),
        DeclareLaunchArgument('trigger_mode/mode', default_value='0'),
        DeclareLaunchArgument('trigger_mode/on_off', default_value='True'),
        DeclareLaunchArgument('trigger_mode/polarity', default_value='0'),
        DeclareLaunchArgument('change_rgb_flag', default_value='True'),
        DeclareLaunchArgument('output_voltage', default_value='False'),
    ]

    tmc_pgr_camera_node = Node(
        package='tmc_pgr_camera',
        executable='tmc_pgr_camera_node',
        name=LaunchConfiguration('node_name'),
        parameters=[{
            'camera_setting_file_path': LaunchConfiguration('camera_setting_file_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'auto_capture_start': LaunchConfiguration('auto_capture_start'),
            'plugin_name': LaunchConfiguration('plugin_name'),
            'plugin_export_tag_name': LaunchConfiguration('plugin_export_tag_name'),
            'property': {
                'brightness': {
                    'on_off': LaunchConfiguration('property/brightness/on_off'),
                    'abs_value': LaunchConfiguration('property/brightness/abs_value'),
                    'one_push': LaunchConfiguration('property/brightness/one_push'),
                    'auto_manual_mode': LaunchConfiguration('property/brightness/auto_manual_mode'),
                },
                'auto_exposure': {
                    'on_off': LaunchConfiguration('property/auto_exposure/on_off'),
                    'abs_value': LaunchConfiguration('property/auto_exposure/abs_value'),
                    'one_push': LaunchConfiguration('property/auto_exposure/one_push'),
                    'auto_manual_mode': LaunchConfiguration('property/auto_exposure/auto_manual_mode'),
                },
                'white_balance': {
                    'on_off': LaunchConfiguration('property/white_balance/on_off'),
                    'value_a': LaunchConfiguration('property/white_balance/value_a'),
                    'value_b': LaunchConfiguration('property/white_balance/value_b'),
                    'one_push': LaunchConfiguration('property/white_balance/one_push'),
                    'auto_manual_mode': LaunchConfiguration('property/white_balance/auto_manual_mode'),
                },
                'shutter': {
                    'on_off': LaunchConfiguration('property/shutter/on_off'),
                    'abs_value': LaunchConfiguration('property/shutter/abs_value'),
                    'one_push': LaunchConfiguration('property/shutter/one_push'),
                    'auto_manual_mode': LaunchConfiguration('property/shutter/auto_manual_mode'),
                },
                'gain': {
                    'on_off': LaunchConfiguration('property/gain/on_off'),
                    'abs_value': LaunchConfiguration('property/gain/abs_value'),
                    'one_push': LaunchConfiguration('property/gain/one_push'),
                    'auto_manual_mode': LaunchConfiguration('property/gain/auto_manual_mode'),
                },
                'trigger_delay': {
                    'on_off': LaunchConfiguration('property/trigger_delay/on_off'),
                    'abs_value': LaunchConfiguration('property/trigger_delay/abs_value'),
                    'abs_control': LaunchConfiguration('property/trigger_delay/abs_control'),
                },
            },
            'format7': {
                'mode': LaunchConfiguration('format7/mode'),
                'offset_x': LaunchConfiguration('format7/offset_x'),
                'offset_y': LaunchConfiguration('format7/offset_y'),
                'width': LaunchConfiguration('format7/width'),
                'height': LaunchConfiguration('format7/height'),
                'pixel_format': LaunchConfiguration('format7/pixel_format'),
            },
            'self_trigger': {
                'io': LaunchConfiguration('self_trigger/io'),
                'pulse_width': LaunchConfiguration('self_trigger/pulse_width'),
                'number_of_pulse': LaunchConfiguration('self_trigger/number_of_pulse'),
                'polarity': LaunchConfiguration('self_trigger/polarity'),
            },
            'software_demosaicing': LaunchConfiguration('software_demosaicing'),
            'video_mode': LaunchConfiguration('video_mode'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'trigger_mode': {
                'mode': LaunchConfiguration('trigger_mode/mode'),
                'on_off': LaunchConfiguration('trigger_mode/on_off'),
                'polarity': LaunchConfiguration('trigger_mode/polarity'),
            },
            'change_rgb_flag': LaunchConfiguration('change_rgb_flag'),
            'output_voltage': LaunchConfiguration('output_voltage'),
            'image_topic_names': LaunchConfiguration('image_topic_names'),
        }],
        output='screen')

    ld = LaunchDescription(
        args_fundamentals
        + args_properties
        + args_format7
        + args_sync_pulse_setting
        + args_other
        + [tmc_pgr_camera_node,]
    )
    return ld

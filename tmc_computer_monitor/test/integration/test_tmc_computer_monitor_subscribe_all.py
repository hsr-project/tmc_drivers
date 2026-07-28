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

import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import yaml


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription(
        [
            Node(
                package="tmc_computer_monitor",
                executable="tmc_computer_monitor_node",
                name="test_target_node",
                output="screen",
                parameters=[
                    {"config_files": "test_computer_diag.yaml", "loop_rate": 0.7}
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestTmcComputerMonitorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_node")
        cls.cpu_start_time = None
        cls.mem_start_time = None
        cls.disk_start_time = None
        cls.cpu_count = 0
        cls.mem_count = 0
        cls.disk_count = 0
        cls.file_count = 0
        cls.cpu_hz = 0
        cls.mem_hz = 0
        cls.subscribe_list = []
        cls.another_prefix_diags = []
        cls.diag_msgs = []
        cls.diag_names = []

        config_file = (
            get_package_share_directory("tmc_computer_monitor")
            + "/config/test_computer_diag.yaml"
        )
        with open(config_file, "r") as f:
            cls.configs = yaml.load(f)

    @classmethod
    def tearDownClass(cls):
        cls.cpu_start_time = None
        cls.mem_start_time = None
        cls.disk_start_time = None
        cls.cpu_count = 0
        cls.mem_count = 0
        cls.disk_count = 0
        cls.file_count = 0
        cls.cpu_hz = 0
        cls.mem_hz = 0
        cls.subscribe_list = []
        cls.another_prefix_diags = []
        cls.diag_msgs = []
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def diag_callback(cls, diag):
        for diag in diag.status:
            cls.diag_msgs.append(diag)
            if not diag.name.startswith("tmc_computer_monitor/"):
                cls.another_prefix_diags.append(diag)
                continue
            computer_key = diag.name[len("tmc_computer_monitor/computer/"):]
            file_key = diag.name[len("tmc_computer_monitor/file/"):]
            if computer_key in cls.configs.keys():
                cls.subscribe_list.append(computer_key)
            elif file_key in cls.configs.keys():
                cls.subscribe_list.append(file_key)
            if diag.name == "tmc_computer_monitor/computer/CPU":
                if cls.cpu_count == 0:
                    cls.cpu_start_time = cls.node.get_clock().now()
                else:
                    elapsed_time = cls.node.get_clock().now() - cls.cpu_start_time
                    elapsed_time_ns = elapsed_time.nanoseconds
                    elapsed_time_s = float(elapsed_time_ns / 1e9)
                    cls.cpu_hz = 1 / float(elapsed_time_s / cls.cpu_count)
                cls.cpu_count += 1
            elif diag.name == "tmc_computer_monitor/computer/Memory":
                if cls.mem_count == 0:
                    cls.mem_start_time = cls.node.get_clock().now()
                else:
                    elapsed_time = cls.node.get_clock().now() - cls.mem_start_time
                    elapsed_time_ns = elapsed_time.nanoseconds
                    elapsed_time_s = float(elapsed_time_ns / 1e9)
                    cls.mem_hz = 1 / float(elapsed_time_s / cls.mem_count)
                cls.mem_count += 1
            elif diag.name == "tmc_computer_monitor/computer/Disk":
                if cls.disk_count == 0:
                    cls.disk_start_time = cls.node.get_clock().now()
                cls.disk_count += 1
            elif diag.name == "tmc_computer_monitor/file/File":
                if cls.file_count == 0:
                    cls.file_start_time = cls.node.get_clock().now()
                cls.file_count += 1

    def test_subscribe_all(self):
        """TESTCASE 3: Confirm if all results can be subscribed"""
        # Wait until all results are subscribed
        self.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diag_callback,
            qos_profile=qos_profile_sensor_data,
        )
        start = self.node.get_clock().now()
        while rclpy.ok() and self.node.get_clock().now() - start < Duration(seconds=60):
            if self.disk_count > 1:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for key in self.configs.keys():
            # Confirm that files with publish_rate set to 0.0 are not published thereafter
            if key == "File":
                self.assertEqual(self.subscribe_list.count("File"), 1)
            else:
                self.assertIn(key, self.subscribe_list, self.diag_msgs)

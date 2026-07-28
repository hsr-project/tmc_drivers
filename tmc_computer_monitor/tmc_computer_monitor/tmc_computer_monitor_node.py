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
from concurrent.futures import Future
import random
import time
from typing import List

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from tmc_computer_monitor.command_monitor_factory import CommandMonitorFactory
import yaml


class ComputerMonitorNode(Node):
    PKG_NAME = "tmc_computer_monitor"

    def __init__(self):
        """Init for every monitor, usually call as a super from child."""
        super().__init__("tmc_computer_monitor_node")
        self.declare_parameter("loop_rate", 1.0)
        self.declare_parameter("prefix", f"{self.PKG_NAME}/")
        self.declare_parameter("config_files", "")

        self.__sleep_duration = 1.0

        self.__prefix = f"{self.PKG_NAME}/"

        self.__monitors = []
        self.__publish_rates = []
        self.__update_once = []
        self.__offset = []
        self.__next_publish_time = []

        self.__config_file_declared_event: Future = Future()
        self.__is_config_file_reloaded = False

        self.__publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        if self.is_parameter_loaded_on_init():
            self.__config_file_declared_event.set_result(True)

    def calculate_next_publish_time(self, duration: float, offset=None):
        if offset is None or offset == 0.0:
            return self.get_clock().now() + Duration(seconds=duration * random.random())
        else:
            return self.get_clock().now() + Duration(seconds=float(offset))

    def get_sleep_duration(self):
        return self.__sleep_duration

    def update_task(self):
        self.setup_publish_time()
        self.update()

    def setup_publish_time(self):
        if self.__is_config_file_reloaded:

            for i in range(len(self.__monitors)):
                if self.__publisher.get_subscription_count() == 0:
                    return

                publish_duration = (
                    1.0 / self.__publish_rates[i]
                    if self.__publish_rates[i] != 0.0
                    else 1.0
                )

                self.__next_publish_time[i] = self.calculate_next_publish_time(
                    publish_duration, self.__offset[i]
                )

            self.__is_config_file_reloaded = False

    def update(self):
        for i in range(len(self.__monitors)):

            if self.__publisher.get_subscription_count() == 0:
                return

            if (
                self.get_clock().now() > self.__next_publish_time[i]
                and not self.__update_once[i]
            ):
                diag = DiagnosticArray()
                diag = self.__monitors[i].get_diag()
                diag.header.stamp = self.get_clock().now().to_msg()

                if self.__publisher.get_subscription_count() == 0:
                    return

                self.__publisher.publish(diag)

                publish_duration = (
                    1.0 / self.__publish_rates[i]
                    if self.__publish_rates[i] != 0.0
                    else 1.0
                )

                while self.__next_publish_time[i] < self.get_clock().now():
                    self.__next_publish_time[i] += Duration(seconds=publish_duration)
                if self.__publish_rates[i] == 0.0:
                    self.__update_once[i] = True

    def parameters_callback(self, params: List[SetParametersResult]):
        self.get_logger().info("parameters callback")
        for param in params:
            if param.name == "prefix":
                self.__prefix = param.value
            if param.name == "loop_rate":
                loop_rate = param.value
                self.get_logger().info(f"Setting loop rate to {loop_rate}")
                self.__rate = self.create_rate(loop_rate)
            if param.name == "config_files":
                config_list = self.load_config_file(param.value)
                self.__monitors = CommandMonitorFactory.generate_monitor(
                    config_list, self.__prefix
                )
                self.__publish_rates = CommandMonitorFactory.generate_publish_rate(
                    config_list
                )
                self.__offset = CommandMonitorFactory.generate_offset(config_list)
                self.__update_once = [False for _ in range(len(self.__publish_rates))]
                self.__next_publish_time = [
                    self.get_clock().now() for _ in range(len(self.__publish_rates))
                ]

                if not self.is_config_file_declared().done():
                    self.__config_file_declared_event.set_result(True)
                self.__is_config_file_reloaded = True

        return SetParametersResult(successful=True)

    def is_config_file_declared(self) -> Future:
        return self.__config_file_declared_event

    def is_parameter_loaded_on_init(self) -> bool:
        config_path = self.get_parameter("config_files")
        prefix = self.get_parameter("prefix")
        loop_rate = self.get_parameter("loop_rate")

        if prefix.type_ == rclpy.parameter.Parameter.Type.NOT_SET:
            self.__prefix = f"{self.PKG_NAME}/"
        else:
            self.__prefix = prefix.value

        if config_path.type_ == rclpy.parameter.Parameter.Type.NOT_SET:
            return False
        else:
            self.get_logger().info(
                f"loop rate: {loop_rate.value}, prefix: {self.__prefix}, config file: {config_path.value}"
            )
            config_list = self.load_config_file(config_path.value)
            self.__sleep_duration = 1.0 / loop_rate.value
            self.__monitors = CommandMonitorFactory.generate_monitor(
                config_list, self.__prefix
            )
            self.__publish_rates = CommandMonitorFactory.generate_publish_rate(
                config_list
            )
            self.__offset = CommandMonitorFactory.generate_offset(config_list)
            self.__update_once = [False for _ in range(len(self.__publish_rates))]
            self.__next_publish_time = [
                self.get_clock().now() for _ in range(len(self.__publish_rates))
            ]
            self.__is_config_file_reloaded = True
            return True

    def load_config_file(self, config_path: str) -> dict:
        try:
            with open(config_path, "r") as f:
                config_list = yaml.safe_load(f)
            return config_list

        except FileNotFoundError as e:
            raise e


def main():
    rclpy.init()
    node = ComputerMonitorNode()
    # Wait until the config file is retrieved
    rclpy.spin_until_future_complete(node, node.is_config_file_declared())
    # Execute the task
    while rclpy.ok():
        node.update_task()
        start = time.time()
        while time.time() - start < node.get_sleep_duration():
            rclpy.spin_once(node, timeout_sec=0.01)

    rclpy.shutdown()

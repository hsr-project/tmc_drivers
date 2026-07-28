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

import importlib
from typing import List
from typing import Union

from .command_monitor import CommandMonitor
from .file_monitor import FileMonitor


class CommandMonitorFactory:

    @staticmethod
    def generate_monitor(
        config: dict, prefix: str
    ) -> List[Union[CommandMonitor,
                    FileMonitor]]:

        monitors = []

        for key in config.keys():
            conf = config[key]
            module_name = conf.get("module")
            if module_name is None:
                module_name = "tmc_computer_monitor"
            try:
                module = importlib.import_module(module_name)
                monitor_class = getattr(module, conf["type"])
                monitor = monitor_class(key, conf, prefix)
            except ModuleNotFoundError:
                print(f"Module {module_name} not found.")
                continue
            monitors.append(monitor)

        return monitors

    @staticmethod
    def generate_publish_rate(config: dict) -> List[float]:

        publish_rates = []

        for key in config.keys():
            conf = config[key]
            rate = conf.get("publish_rate")
            if rate is not None:
                publish_rates.append(rate)
            else:
                publish_rates.append(0.0)

        return publish_rates

    @staticmethod
    def generate_offset(config: dict) -> List[float]:

        offsets = []

        for key in config.keys():
            conf = config[key]
            offset = conf.get("offset")
            if offset is not None:
                offsets.append(offset)
            else:
                offsets.append(0.0)

        return offsets

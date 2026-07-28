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
import re

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from .command_monitor import CommandMonitor


class SensorsCommandMonitor(CommandMonitor):
    """A class to diagnose temperature and FAN rotation using the sensors command"""

    _command = ["/usr/bin/sensors", "-u", "-A"]
    _item_info = {"temp": {"unit": "°C"}, "in": {"unit": "V"}, "fan": {"unit": "rpm"}}

    def _parse(self, result):
        rows = [row for row in result if row != ""]
        prefix = ""
        # Summarize the information into devs once
        devs = {}
        for row in rows:
            # If there is a line that does not contain ":", make that line the prefix
            if ":" not in row:
                prefix = row
                continue
            # Target only lines that start with a space
            if not row.startswith(" "):
                continue
            # Separate by ":"
            key, value = row.split(":")
            # Separate by "_"
            dev, value_type = key.strip().split("_", 1)
            # temp1_input: 29.000 becomes
            # dev=temp1, value_type=input, value=29.000
            full_dev_name = prefix + "/" + dev
            if full_dev_name not in devs:
                devs[full_dev_name] = {}
            devs[full_dev_name][value_type] = float(value)
        values = []
        max_temp = 0.0
        max_rpm = 0.0
        for dev_name, dev_info in devs.items():
            for key, value in dev_info.items():
                if key == "input":
                    # Get the maximum temperature (ensuring it is not in a fault state)
                    if "temp" in dev_name and "fault" not in dev_info:
                        max_temp = max(max_temp, value)
                    # Get the maximum FAN rotation speed
                    elif "fan" in dev_name:
                        max_rpm = max(max_rpm, value)
                unit = self._item_info[re.sub(r'\d+$', '', dev_name.split("/")[-1])]["unit"]
                for item in self._config["items"]:
                    if item in dev_name:
                        values.append(
                            KeyValue(
                                key="{0}/{1}".format(dev_name, key),
                                value="{0} {1}".format(value, unit),
                            )
                        )
        msg = "Temp: {0} °C/ Fan speed: {1} rpm".format(max_temp, max_rpm)
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

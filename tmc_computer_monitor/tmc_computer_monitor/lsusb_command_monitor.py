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

import os
import re

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from .command_monitor import CommandMonitor

DIAG_PREFIX = "tmc_computer_monitor/"


class LsusbCommandMonitor(CommandMonitor):
    """A class to diagnose USB connection status using the lsusb command"""

    def __init__(self, name: str, config: dict, prefix=DIAG_PREFIX):
        """Initialize, read config and parse informations"""
        super(LsusbCommandMonitor, self).__init__(name, config, prefix)
        self.has_custom_config = False
        self._serial = False
        self._command = ['/usr/bin/lsusb']
        self._command_config(config)

    def _command_config(self, config: dict) -> None:
        """Create command according to config"""
        if "device" in config:
            self.has_custom_config = True
            self._command = ['/usr/bin/lsusb', '-d', config['device']]

        if 'grep_match' in config:
            self.has_custom_config = True
            env_matched = [re.match(r'^\$\{(.*)\}$', config['grep_match'][i]) for i in range(len(config['grep_match']))]
            self._command.append('-v')
            self._shell_command = ' '.join(self._command)
            for i in range(len(config['grep_match'])):
                if env_matched[i] is not None:
                    serial_value = os.environ.get(env_matched[i].group(1), '')
                else:
                    serial_value = config['grep_match'][i]
                self._shell_command += f' | grep {serial_value}'

    def _parse(self, result) -> DiagnosticStatus:
        try:
            if len(result) == 0 or result[0].strip() == "":
                raise RuntimeError()
            if not self.has_custom_config:
                return self._parse_default(result)
            elif 'device' in self._config:
                if ((not self._serial and self._config['device'] in result[0])
                        or (self._serial and result)):
                    return DiagnosticStatus(level=DiagnosticStatus.OK,
                                            message='OK', hardware_id=self._hardware_id)
            elif 'grep_match' in self._config:
                grep_result = [device_info
                               for device_info in result
                               for grep_keyword in self._config["grep_match"]
                               if grep_keyword in device_info
                               ]
                if len(grep_result) != 0:
                    values = []
                    values.append(KeyValue(key="grep keyword ", value=str(self._config['grep_match'])))
                    values.append(KeyValue(key="lsusb grep", value=str(grep_result)))
                    return DiagnosticStatus(level=DiagnosticStatus.OK,
                                            message='OK',
                                            hardware_id=self._hardware_id,
                                            values=values,)
            return DiagnosticStatus(level=DiagnosticStatus.OK, message='OK')
        except RuntimeError:
            if 'grep_match' in self._config or 'device' in self._config:
                raise RuntimeError("Device not found.")
            else:
                raise RuntimeError("No device is connected")

    def _parse_default(self, result) -> DiagnosticStatus:
        values = []
        for row in result:
            bus = row[4:7]
            device = row[15:18]
            values.append(KeyValue(key="{0}/{1}".format(bus, device), value=row[20:]))
        msg = "{0} devices connected".format(len(result))
        return DiagnosticStatus(level=DiagnosticStatus.OK, values=values, message=msg)

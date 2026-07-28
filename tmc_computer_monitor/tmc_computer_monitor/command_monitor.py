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
import subprocess
import sys

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

DIAG_PREFIX = "tmc_computer_monitor/"


class CommandMonitor(object):
    """Base class for executing some commands to monitor diagnostics"""

    _command = None
    _shell_command = None

    def __init__(self, name: str, config: dict, prefix=DIAG_PREFIX):
        """Constructor"""
        # Check the config
        self._hostname = os.uname()[1]
        self._name = name
        self._config = config
        self._hardware_id = self._hostname
        self._check_config()
        self._prefix = prefix

    def get_diag(self) -> DiagnosticArray:
        """Execute a command, parse the result, and return a DiagnosticArray"""
        command_result = ""
        diag = DiagnosticArray()
        status = None
        try:
            command_result = self._execute_command()
            # check if we have to decode string
            try:
                command_result = command_result.decode()
            except (UnicodeDecodeError, AttributeError):
                pass
            result = command_result.splitlines()
            status = self._parse(result)
        except Exception as err:
            # Whenever command_result cannot be read or parsed
            values = []
            values.append(KeyValue(key="exception", value=err.__class__.__name__))
            values.append(KeyValue(key="message", value=str(err)))
            values.append(KeyValue(key="command_result", value=command_result))
            status = DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                message="failed to update diagnostic for {0}".format(self._name),
                values=values,
                hardware_id=self._hardware_id
            )
        if not status:
            status = DiagnosticStatus()
        status.hardware_id = self._hardware_id
        if not isinstance(status, list):
            status.name = self._prefix + "computer/" + self._name
            diag.status = [status]
        else:
            diag.status = status
        return diag

    def _execute_command(self):
        """Execute a command and return the result

        Mock this method during test execution
        """
        command = None
        use_shell = False
        # If shell_command is set, execute with shell=True
        if self._shell_command is not None:
            command = self._shell_command
            use_shell = True
        elif self._command is not None:
            command = self._command
        # Ensure command is a string or list
        if not isinstance(command, (str, list)):
            raise TypeError(f"Command must be a string or a list, got {type(command).__name__}")

        p = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={"LANG": "C"},
            shell=use_shell,
        )
        stdout, stderr = p.communicate()
        retcode = p.returncode
        if retcode != 0 and "grep" not in command:
            err = stderr.decode('utf-8').strip()
            raise RuntimeError(
                "Failed execute {0}, retcode={1}, stderr={2}".format(command, retcode, err)
            )
        if sys.version_info.major == 3:
            return stdout.decode()
        else:
            return stdout

    def _get_config(self, key):
        if key not in self._config:
            raise RuntimeError("{0} is not defined in config file".format(key))
        return self._config[key]

    def _check_config(self) -> None:

        if "items" in self._config:
            for key in self._config["items"]:
                if key not in self._item_info:
                    raise RuntimeError(
                        "Unknown item: {0} in {1}".format(key, self._name)
                    )
        # check for hardware_id
        if "hardware_id" in self._config:
            if not isinstance(self._config["hardware_id"], str):
                raise ValueError("hardware_id must be a string")
            self._hardware_id = self._config["hardware_id"]
        # Check if pid and vid are hexadecimal values and and 4 characteres
        if "device" in self._config:
            def is_hexadecimal(s):
                return bool(re.match(r'^[0-9a-fA-F]{4}$', s))
            if isinstance(self._config["device"], str):
                pid_vid = self._config["device"].split(":")  # spit according to : to separate pid and vid
                if len(pid_vid) == 2:
                    for value in pid_vid:
                        if not is_hexadecimal(value):
                            raise AttributeError("PID:VID value is invalid. PID:VID must be hexadecimales values")
        # check for target
        if "target" in self._config:
            if not isinstance(self._config["target"], list):
                raise ValueError("target of {0} is not list".format(self._name))

    def _parse(self, result):
        """Implement a method in the subclass to parse the command execution result and return a DiagnosticStatus"""
        raise NotImplementedError()

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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

DIAG_PREFIX = "tmc_computer_monitor/"


class FileMonitor(object):
    """Class to check and diagnose the contents of a file"""

    def __init__(self, name, config, prefix=DIAG_PREFIX):
        """Constructor"""
        self._hostname = os.uname()[1]
        # Check config
        self._name = name
        self._config = config
        self._prefix = prefix

    def _get_config(self, key):
        if key not in self._config:
            raise RuntimeError("{0} is not defined in config file".format(key))
        return self._config[key]

    def _file_open(self, filename):
        with open(filename, "r") as f:
            return f.readlines()

    def get_diag(self):
        status_list = []
        file_configs = self._get_config("files")
        for config in file_configs:
            name = config
            filename = file_configs[config][0]["path"]
            contains = file_configs[config][0].get("contain", [])
            notcontains = file_configs[config][0].get("notcontain", [])
            error_message = file_configs[config][0].get("error_message", "")
            status = DiagnosticStatus()
            status.name = self._prefix + "file/" + name
            status.hardware_id = self._hostname
            status.level = DiagnosticStatus.OK
            status.message = "OK"

            # Open the file
            try:
                contents = self._file_open(filename)
            except Exception:
                # Message for when the file is not found is standardized
                status.level = DiagnosticStatus.ERROR
                status.message = "No such file: {0}".format(filename)
                status.values.append(KeyValue(key="Path", value=filename))
                status_list.append(status)
                continue
            status.values.append(KeyValue(key="Path", value=filename))
            for no, line in enumerate(contents):
                # Check contain
                for contain in contains[:]:
                    if contain in line:
                        contains.remove(contain)
                # Check notcontain
                for notcontain in notcontains:
                    if notcontain in line:
                        # When an expected non-contained target is found
                        status.level = DiagnosticStatus.ERROR
                        if len(error_message) == 0:
                            status.message = "File contains an invalid content"
                        else:
                            status.message = error_message
                        status.values.append(
                            KeyValue(
                                key="NotContain",
                                value="'{0}' in line:{1}".format(notcontain, no + 1),
                            )
                        )
            # When not all contain targets are included
            if len(contains) > 0:
                status.level = DiagnosticStatus.ERROR
                if error_message == "":
                    status.message = "File does not contain a necessary line"
                else:
                    status.message = error_message
                status.values.append(
                    KeyValue(key="Contain", value="{0} not contain".format(contains))
                )
            status_list.append(status)
        diag = DiagnosticArray(status=status_list)
        return diag

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

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from .command_monitor import CommandMonitor


class PsCommandMonitor(CommandMonitor):
    """A class to diagnose processes using the ps command"""

    _command = ["/bin/ps", "aux"]
    _item_info = {
        "UserID": {"field": "USER", "name": "User ID"},
        "ProcessID": {"field": "PID", "name": "Process ID"},
        "UseCpuRate": {"field": "%CPU", "name": "Use CPU rate[%]"},
        "UseMemoryRate": {"field": "%MEM", "name": "Use memory rate[%]"},
        "UseVirtualMemorySize": {"field": "VSZ", "name": "Use virtual memory size[kb]"},
        "UsePhisicalMemorySize": {
            "field": "RSS",
            "name": "Use phisical memory size[kb]",
        },
        "TypeOfTerminal": {"field": "TTY", "name": "Type of terminal"},
        "ProcessStatus": {"field": "STAT", "name": "Process status"},
        "ProcessStartTime": {"field": "START", "name": "process start time"},
        "ExecutionTime": {"field": "TIME", "name": "Execution time"},
        "ExecutionCommand": {"field": "COMMAND", "name": "Execution command"},
    }

    def _check_config(self):
        super(PsCommandMonitor, self)._check_config()
        self._num_to_disp = self._get_config("num_to_disp")
        self._sort_key = self._get_config("sort_key")

    def _parse(self, result):
        rows = [row.split(None, 10) for row in result if row != ""]
        names = rows[0]
        sort_index = names.index(self._item_info[self._sort_key]["field"])
        # Sort by the target key and extract only the necessary amount
        rows = sorted(rows[1:], key=lambda x: float(x[sort_index]), reverse=True)
        # Display only the specified amount
        values = []
        for no, row in enumerate(rows[: self._num_to_disp]):
            for key in self._config["items"]:
                idx = names.index(self._item_info[key]["field"])
                values.append(
                    KeyValue(
                        key="Process {0} {1}".format(
                            no + 1, self._item_info[key]["name"]
                        ),
                        value=row[idx],
                    )
                )
        msg = "{0}={1} [{2}]".format(
            self._item_info[self._sort_key]["name"], rows[0][sort_index], rows[0][10]
        )
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

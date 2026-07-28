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


class MpstatCommandMonitor(CommandMonitor):
    """A class to diagnose CPU load status with mpstat"""

    _command = ["/usr/bin/mpstat", "-P", "ALL", "1", "1"]
    _item_info = {
        "User": {"index": 2, "name": "User"},
        "Nice": {"index": 3, "name": "Nice"},
        "Sys": {"index": 4, "name": "Sys"},
        "IOwait": {"index": 5, "name": "IOwait"},
        "IRQ": {"index": 6, "name": "IRQ"},
        "SoftIRQ": {"index": 7, "name": "SoftIRQ"},
        "Steal": {"index": 8, "name": "Steal"},
        "Guest": {"index": 9, "name": "Guest"},
        "Gnice": {"index": 10, "name": "Gnice"},
        "Idle": {"index": 11, "name": "Idle"},
    }

    def _parse(self, result):
        values = []
        # Collect only the parts labeled with Average
        rows = [row.split() for row in result if row.startswith("Average:")]
        # Average:     all    1.00    2.00    3.00    4.00    5.00    6.00    7.00    8.00    9.00   10.00  # noqa
        for row in rows[1:]:
            for key in self._config["items"]:
                conf = self._item_info[key]
                values.append(
                    KeyValue(
                        key="CPU {0}: {1}".format(row[1], conf["name"]),
                        value=row[conf["index"]],
                    )
                )
        msg = "user={0}/sys={1}/io={2}/idle={3}".format(
            rows[1][2], rows[1][4], rows[1][5], rows[1][11]
        )
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

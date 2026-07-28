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


class DfCommandMonitor(CommandMonitor):
    """A class to diagram disk usage with the df command"""

    _command = ["/bin/df"]
    _item_info = {
        "Filesystem": {"index": 0, "name": "Filesystem"},
        "Size": {"index": 1, "name": "Size"},
        "Used": {"index": 2, "name": "Used"},
        "Avail": {"index": 3, "name": "Avail"},
        "Use%": {"index": 4, "name": "Use%"},
    }

    def _check_config(self):
        super(DfCommandMonitor, self)._check_config()
        self._target = self._get_config("target")

    def _parse(self, result):
        rows = [row.split() for row in result[1:] if row != ""]
        values = []
        target_index = -1
        for row_index, row in enumerate(rows):
            target = row[5]
            for key in self._config["items"]:
                conf = self._item_info[key]
                # Keep track of target_index
                if target in self._target and target_index < 0:
                    target_index = row_index
                values.append(
                    KeyValue(
                        key="{0}: {1}".format(target, conf["name"]),
                        value=row[conf["index"]],
                    )
                )
        if target_index >= 0:
            msg = "{0}: Used {1}".format(rows[target_index][5], rows[target_index][4])
        else:
            msg = "{0} not found".format(", ".join(self._target))
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

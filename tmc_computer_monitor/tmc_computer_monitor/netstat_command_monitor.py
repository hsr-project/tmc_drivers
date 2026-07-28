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


class NetstatCommandMonitor(CommandMonitor):
    """A class for diagnosing network communication status using the netstat command"""

    _command = ["netstat", "-i"]
    # Write the display name for netstat -i in short form
    _item_info = {
        "Interface": {"index": 0, "name": "Interface", "short": "Iface"},
        "MTU": {"index": 1, "name": "MTU", "short": "MTU"},
        "ReceiveOK": {"index": 2, "name": "Receive OK", "short": "RX-OK"},
        "ReceiveERR": {"index": 3, "name": "Receive ERR", "short": "RX-ERR"},
        "ReceiveDRP": {"index": 4, "name": "Receive DRP", "short": "RX-DRP"},
        "ReceiveOVR": {"index": 5, "name": "Receive OVR", "short": "RX-OVR"},
        "SendOK": {"index": 6, "name": "Send OK", "short": "TX-OK"},
        "SendERR": {"index": 7, "name": "Send ERR", "short": "TX-ERR"},
        "SendDRP": {"index": 8, "name": "Send DRP", "short": "TX-DRP"},
        "SendOVR": {"index": 9, "name": "Send OVR", "short": "TX-OVR"},
        "Flg": {"index": 10, "name": "Flg", "short": "Flg"},
    }

    def _check_config(self):
        super(NetstatCommandMonitor, self)._check_config()
        self._target = self._get_config("target")

    def _parse(self, result):
        # Retrieve the names of each information item in the second line of netstat -i
        status_names = result[1].split()
        rows = []
        for line in result[2:]:
            if line == "":
                continue
            elif "no statistics available" in line:
                continue
            else:
                rows.append(line.split())
        # Retrieve all shorts from _item_info
        short_names = []
        for item in self._item_info.values():
            short_names.append(item["short"])
        # Remove columns from status_names if they are not in short_names
        for name_index, name in enumerate(status_names):
            if name not in short_names:
                for i in range(len(rows)):
                    rows[i].pop(name_index)

        values = []
        target_index = -1
        for row_index, row in enumerate(rows):
            target = row[0]
            for key in self._config["items"]:
                conf = self._item_info[key]
                # Retrieve when the interface name matches completely or partially
                for target_name in self._target:
                    if target in target_name and target_index < 0:
                        target_index = row_index
                        # Change to the real name as the display name for netstat -i may be abbreviated
                        rows[target_index][0] = target_name
                        target = target_name
                values.append(
                    KeyValue(
                        key="{0}: {1}".format(target, conf["name"]),
                        value=row[conf["index"]],
                    )
                )
        if target_index >= 0:
            msg = "{0}: RXOK/{1} TXOK/{2}".format(
                rows[target_index][0], rows[target_index][2], rows[target_index][6]
            )
        else:
            msg = "{0} not found".format(", ".join(self._target))
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

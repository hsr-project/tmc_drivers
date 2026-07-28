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


class IwconfigCommandMonitor(CommandMonitor):
    """A class to diagnose the status of the wireless LAN interface using the iwconfig command"""

    _command = ["/sbin/iwconfig"]
    _item_info = {
        "IEEE": {"field": "IEEE", "name": "IEEE"},
        "ESSID": {"field": "ESSID", "name": "ESSID"},
        "Nickname": {"field": "Nickname", "name": "Nickname"},
        "Mode": {"field": "Mode", "name": "Mode"},
        "Frequency": {"field": "Frequency", "name": "Frequency"},
        "AccessPointMac": {"field": "Access Point", "name": "Access point MAC"},
        "BitRate": {"field": "Bit Rate", "name": "Bit rate"},
        "TxPower": {"field": "Tx-Power", "name": "Tx power"},
        "Sensitivity": {"field": "Sensitivity", "name": "Sensitivity"},
        "RetryMinLimit": {"field": "Retry min limit", "name": "Retry min limit"},
        "RTSThr": {"field": "RTS thr", "name": "RTS thr"},
        "FragmentThr": {"field": "Fragment thr", "name": "Fragment thr"},
        "PowerManagement": {"field": "Power Management", "name": "Power management"},
        "LinkQuality": {"field": "Link Quality", "name": "Link quality"},
        "SignalLevel": {"field": "Signal level", "name": "Signal level"},
        "NoiseLevel": {"field": "Noise level", "name": "Noise level"},
        "RxInvalidNwid": {"field": "Rx invalid nwid", "name": "Rx invalid nwid"},
        "InvalidCrypt": {"field": "invalid crypt", "name": "Invalid crypt"},
        "RxInvalidFrag": {"field": "Rx invalid frag", "name": "Rx invalid frag"},
        "TxExcessiveRetries": {
            "field": "Tx excessive retries",
            "name": "Tx excessive retries",
        },
        "InvalidMisc": {"field": "Invalid misc", "name": "Invalid misc"},
        "MissedBeacon": {"field": "Missed beacon", "name": "Missed beacon"},
    }

    def _check_config(self):
        super(IwconfigCommandMonitor, self)._check_config()

    def _parse(self, result):
        rows = []
        for line in result:
            if line == "":
                continue
            elif "no wireless extensions." in line:
                continue
            else:
                rows.append(line)
        if len(rows) > 9:
            raise RuntimeError("Multiple wireless networks are not supported")
        lists = []
        values = []
        for row in rows:
            # After splitting by "  ", create a list by applying the following process to non-empty elements
            # Remove leading and trailing spaces, then split by ":" or "="
            tmp = [
                re.split("[:=]", item.strip(), 1)
                for item in row.split("  ")
                if item.strip() != ""
            ]
            lists.extend(tmp)
        # Extract only IEEE
        ieee = lists[1][0]
        lists = {row[0]: row[1] for row in lists[2:]}
        for key in self._config["items"]:
            if self._item_info[key]["field"] in lists:
                values.append(
                    KeyValue(
                        key=self._item_info[key]["name"],
                        value=lists[self._item_info[key]["field"]],
                    )
                )
            elif self._item_info[key]["field"] in ieee:
                values.append(KeyValue(key=self._item_info[key]["name"], value=ieee))

        if lists["ESSID"] == "off/any":
            msg = "Not connected"
        else:
            msg = "{0} Bit Rate: {1}  Signal: {2}".format(
                lists["ESSID"], lists["Bit Rate"], lists["Signal level"]
            )
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

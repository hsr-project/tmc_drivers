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


def read_status(pid: int) -> dict:
    status_file = f"/proc/{pid}/status"
    if not os.path.exists(status_file):
        return None

    rss = 0
    locked = 0
    name = ""

    try:
        with open(status_file, "r") as f:
            for line in f:
                if line.startswith("Name:"):
                    name = line.split()[1]
                elif line.startswith("VmRSS:"):
                    rss = int(line.split()[1])  # kB
                elif line.startswith("VmLck:") or line.startswith("Locked:"):
                    locked = int(line.split()[1])  # kB
    except Exception:
        return None

    return {"pid": pid, "name": name, "rss": rss, "locked": locked}


class ProcMemoryMonitor:
    """A class to check memory usage from /proc/{pid} and generate diagrams"""

    def __init__(self, name: str, config: dict, prefix: str):
        self._hostname = os.uname()[1]
        self._name = name
        self._prefix = prefix
        self._top_n = config.get("top_n", 5)

    def get_diag(self) -> DiagnosticArray:
        status = DiagnosticStatus()
        status.name = self._prefix + "computer/" + self._name
        status.hardware_id = self._hostname
        status.level = DiagnosticStatus.OK
        status.message = f'Top {self._top_n} processes by RSS and locked memory usage'

        processes = []
        for pid in os.listdir("/proc"):
            if pid.isdigit():
                info = read_status(pid)
                if info:
                    processes.append(info)

        # Error handling is omitted since self._top_n is unlikely to be set to 100 or 200

        top_rss = sorted(processes, key=lambda x: x["rss"], reverse=True)[:self._top_n]
        for i, rss in enumerate(top_rss, start=1):
            # Processes with RSS of 0 should not appear at the top, so the processing is omitted
            status.values.append(KeyValue(
                key=f'RSS Rank {i}',
                value=f'PID {rss["pid"]}  RSS {rss["rss"]} kB  {rss["name"]}'))

        top_locked = sorted(processes, key=lambda x: x["locked"], reverse=True)[:self._top_n]
        for i, locked in enumerate(top_locked, start=1):
            key = f'Locked Rank {i}'
            if locked["locked"] == 0:
                status.values.append(KeyValue(key=key, value='No locked memory'))
            else:
                value = f'PID {locked["pid"]}  Locked {locked["locked"]} kB  {locked["name"]}'
                status.values.append(KeyValue(key=key, value=value))
        return DiagnosticArray(status=[status])

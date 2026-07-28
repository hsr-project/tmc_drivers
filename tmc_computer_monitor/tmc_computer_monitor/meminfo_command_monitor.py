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


class MeminfoCommandMonitor(CommandMonitor):
    """A class to diagnose memory usage with /proc/meminfo"""

    _command = ["/bin/cat", "/proc/meminfo"]
    _item_info = {
        "MemTotal": {"field": "MemTotal", "name": "MemTotal"},
        "MemFree": {"field": "MemFree", "name": "MemFree"},
        "MemAvailable": {"field": "MemAvailable", "name": "MemAvailable"},
        "Buffers": {"field": "Buffers", "name": "Buffers"},
        "Cached": {"field": "Cached", "name": "Cached"},
        "SwapCached": {"field": "SwapCached", "name": "SwapCached"},
        "Active": {"field": "Active", "name": "Active"},
        "Inactive": {"field": "Inactive", "name": "Inactive"},
        "Active(anon)": {"field": "Active(anon)", "name": "Active(anon)"},
        "Inactive(anon)": {"field": "Inactive(anon)", "name": "Inactive(anon)"},
        "Active(file)": {"field": "Active(file)", "name": "Active(file)"},
        "Inactive(file)": {"field": "Inactive(file)", "name": "Inactive(file)"},
        "Unevictable": {"field": "Unevictable", "name": "Unevictable"},
        "Mlocked": {"field": "Mlocked", "name": "Mlocked"},
        "SwapTotal": {"field": "SwapTotal", "name": "SwapTotal"},
        "SwapFree": {"field": "SwapFree", "name": "SwapFree"},
        "Dirty": {"field": "Dirty", "name": "Dirty"},
        "Writeback": {"field": "Writeback", "name": "Writeback"},
        "AnonPages": {"field": "AnonPages", "name": "AnonPages"},
        "Mapped": {"field": "Mapped", "name": "Mapped"},
        "Shmem": {"field": "Shmem", "name": "Shmem"},
        "Slab": {"field": "Slab", "name": "Slab"},
        "SReclaimable": {"field": "SReclaimable", "name": "SReclaimable"},
        "SUnreclaim": {"field": "SUnreclaim", "name": "SUnreclaim"},
        "KernelStack": {"field": "KernelStack", "name": "KernelStack"},
        "PageTables": {"field": "PageTables", "name": "PageTables"},
        "NFS_Unstable": {"field": "NFS_Unstable", "name": "NFS_Unstable"},
        "Bounce": {"field": "Bounce", "name": "Bounce"},
        "WritebackTmp": {"field": "WritebackTmp", "name": "WritebackTmp"},
        "CommitLimit": {"field": "CommitLimit", "name": "CommitLimit"},
        "Committed_AS": {"field": "Committed_AS", "name": "Committed_AS"},
        "VmallocTotal": {"field": "VmallocTotal", "name": "VmallocTotal"},
        "VmallocUsed": {"field": "VmallocUsed", "name": "VmallocUsed"},
        "VmallocChunk": {"field": "VmallocChunk", "name": "VmallocChunk"},
        "HardwareCorrupted": {
            "field": "HardwareCorrupted",
            "name": "HardwareCorrupted",
        },
        "AnonHugePages": {"field": "AnonHugePages", "name": "AnonHugePages"},
        "HugePages_Total": {"field": "HugePages_Total", "name": "HugePages_Total"},
        "HugePages_Free": {"field": "HugePages_Free", "name": "HugePages_Free"},
        "HugePages_Rsvd": {"field": "HugePages_Rsvd", "name": "HugePages_Rsvd"},
        "HugePages_Surp": {"field": "HugePages_Surp", "name": "HugePages_Surp"},
        "Hugepagesize": {"field": "Hugepagesize", "name": "Hugepagesize"},
        "DirectMap4k": {"field": "DirectMap4k", "name": "DirectMap4k"},
        "DirectMap2M": {"field": "DirectMap2M", "name": "DirectMap2M"},
        "DirectMap1G": {"field": "DirectMap1G", "name": "DirectMap1G"},
    }

    def _parse(self, result):
        if len(result) < 40:
            raise RuntimeError("Command result is insufficient")
        # Separate by :
        rows = [row.split(":", 1) for row in result if row != ""]
        # Convert to key/value
        rows = {row[0].strip(): row[1].strip() for row in rows}
        values = []
        for key in self._config["items"]:
            if self._item_info[key]["field"] in rows:
                values.append(
                    KeyValue(
                        key=self._item_info[key]["name"],
                        value=rows[self._item_info[key]["field"]],
                    )
                )

        msg = "Available {0:3.1f}%".format(
            (float(rows["MemAvailable"][:-3]) / float(rows["MemTotal"][:-3]) * 100.0)
        )
        return DiagnosticStatus(level=DiagnosticStatus.OK, message=msg, values=values)

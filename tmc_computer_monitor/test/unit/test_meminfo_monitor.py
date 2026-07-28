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

import unittest
import unittest.mock as mock

from diagnostic_msgs.msg import DiagnosticStatus
import tmc_computer_monitor


class TestMeminfoCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 10: Specify all items and check the normal case (include the actual command result)"""
        m = tmc_computer_monitor.MeminfoCommandMonitor(
            "Memory",
            {
                "items": [
                    "MemTotal",
                    "MemFree",
                    "MemAvailable",
                    "Buffers",
                    "Cached",
                    "SwapCached",
                    "Active",
                    "Inactive",
                    "Active(anon)",
                    "Inactive(anon)",
                    "Active(file)",
                    "Inactive(file)",
                    "Unevictable",
                    "Mlocked",
                    "SwapTotal",
                    "SwapFree",
                    "Dirty",
                    "Writeback",
                    "AnonPages",
                    "Mapped",
                    "Shmem",
                    "Slab",
                    "SReclaimable",
                    "SUnreclaim",
                    "KernelStack",
                    "PageTables",
                    "NFS_Unstable",
                    "Bounce",
                    "WritebackTmp",
                    "CommitLimit",
                    "Committed_AS",
                    "VmallocTotal",
                    "VmallocUsed",
                    "VmallocChunk",
                    "HardwareCorrupted",
                    "AnonHugePages",
                    "HugePages_Total",
                    "HugePages_Free",
                    "HugePages_Rsvd",
                    "HugePages_Surp",
                    "Hugepagesize",
                    "DirectMap4k",
                    "DirectMap2M",
                    "DirectMap1G",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""MemTotal:        8097532 kB
MemFree:         2652496 kB
MemAvailable:    6096964 kB
Buffers:          482852 kB
Cached:          2969324 kB
SwapCached:            0 kB
Active:          3420152 kB
Inactive:        1351960 kB
Active(anon):    1344704 kB
Inactive(anon):   107800 kB
Active(file):    2075448 kB
Inactive(file):  1244160 kB
Unevictable:        1772 kB
Mlocked:            1772 kB
SwapTotal:       8309756 kB
SwapFree:        8309756 kB
Dirty:                24 kB
Writeback:             0 kB
AnonPages:       1321900 kB
Mapped:           433604 kB
Shmem:            132572 kB
Slab:             499000 kB
SReclaimable:     432884 kB
SUnreclaim:        66116 kB
KernelStack:       10256 kB
PageTables:        44736 kB
NFS_Unstable:          0 kB
Bounce:                0 kB
WritebackTmp:          0 kB
CommitLimit:    12358520 kB
Committed_AS:    5861400 kB
VmallocTotal:   34359738367 kB
VmallocUsed:           0 kB
VmallocChunk:          0 kB
HardwareCorrupted:     0 kB
AnonHugePages:    700416 kB
CmaTotal:              0 kB
CmaFree:               0 kB
HugePages_Total:       0
HugePages_Free:        0
HugePages_Rsvd:        0
HugePages_Surp:        0
Hugepagesize:       2048 kB
DirectMap4k:      328488 kB
DirectMap2M:     7983104 kB
DirectMap1G:           0 kB
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Available 75.3%")
        self.assertEqual(len(status.values), 44)
        # Verify only the first 3 and the last 3
        self.assertEqual(status.values[0].key, "MemTotal")
        self.assertEqual(status.values[0].value, "8097532 kB")
        self.assertEqual(status.values[1].key, "MemFree")
        self.assertEqual(status.values[1].value, "2652496 kB")
        self.assertEqual(status.values[2].key, "MemAvailable")
        self.assertEqual(status.values[2].value, "6096964 kB")
        self.assertEqual(status.values[41].key, "DirectMap4k")
        self.assertEqual(status.values[41].value, "328488 kB")
        self.assertEqual(status.values[42].key, "DirectMap2M")
        self.assertEqual(status.values[42].value, "7983104 kB")
        self.assertEqual(status.values[43].key, "DirectMap1G")
        self.assertEqual(status.values[43].value, "0 kB")

    def test_ok_part_items(self):
        """TESTCASE 11: Specify some items and check the normal case"""
        m = tmc_computer_monitor.MeminfoCommandMonitor(
            "Memory",
            {
                "items": [
                    "MemTotal",
                    "MemFree",
                    "MemAvailable",
                    "Buffers",
                    "Cached",
                    "SwapTotal",
                    "SwapFree",
                    "DirectMap2M",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""MemTotal:        8097532 kB
MemFree:         2652496 kB
MemAvailable:    6096964 kB
Buffers:          482852 kB
Cached:          2969324 kB
SwapCached:            0 kB
Active:          3420152 kB
Inactive:        1351960 kB
Active(anon):    1344704 kB
Inactive(anon):   107800 kB
Active(file):    2075448 kB
Inactive(file):  1244160 kB
Unevictable:        1772 kB
Mlocked:            1772 kB
SwapTotal:       8309756 kB
SwapFree:        8309756 kB
Dirty:                24 kB
Writeback:             0 kB
AnonPages:       1321900 kB
Mapped:           433604 kB
Shmem:            132572 kB
Slab:             499000 kB
SReclaimable:     432884 kB
SUnreclaim:        66116 kB
KernelStack:       10256 kB
PageTables:        44736 kB
NFS_Unstable:          0 kB
Bounce:                0 kB
WritebackTmp:          0 kB
CommitLimit:    12358520 kB
Committed_AS:    5861400 kB
VmallocTotal:   34359738367 kB
VmallocUsed:           0 kB
VmallocChunk:          0 kB
HardwareCorrupted:     0 kB
AnonHugePages:    700416 kB
CmaTotal:              0 kB
CmaFree:               0 kB
HugePages_Total:       0
HugePages_Free:        0
HugePages_Rsvd:        0
HugePages_Surp:        0
Hugepagesize:       2048 kB
DirectMap4k:      328488 kB
DirectMap2M:     7983104 kB
DirectMap1G:           0 kB
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Available 75.3%")
        self.assertEqual(len(status.values), 8)

    def test_ng_invalid_config_item(self):
        """TESTCASE 12: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.MeminfoCommandMonitor("Memory", {"items": ["dummy"]})
        self.assertEqual(str(ex.exception), "Unknown item: dummy in Memory")

    def test_ng_insufficient_result(self):
        """TESTCASE 13: Unable to parse the command execution result (insufficient result)"""
        m = tmc_computer_monitor.MeminfoCommandMonitor(
            "Memory",
            {
                "items": [
                    "MemTotal",
                    "MemFree",
                    "MemAvailable",
                    "SwapTotal",
                    "SwapFree",
                    "DirectMap2M",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = """MemTotal:        8097532 kB
MemFree:         2652496 kB
MemAvailable:    6096964 kB
Buffers:          482852 kB
Cached:          2969324 kB
SwapCached:            0 kB
Active:          3420152 kB
Inactive:        1351960 kB
Active(anon):    1344704 kB
Inactive(anon):   107800 kB
Active(file):    2075448 kB
Inactive(file):  1244160 kB
Unevictable:        1772 kB
Mlocked:            1772 kB
SwapTotal:       8309756 kB
SwapFree:        8309756 kB
Dirty:                24 kB
Writeback:             0 kB
AnonPages:       1321900 kB
Mapped:           433604 kB
Shmem:            132572 kB
Slab:             499000 kB
SReclaimable:     432884 kB
SUnreclaim:        66116 kB
KernelStack:       10256 kB
PageTables:        44736 kB
NFS_Unstable:          0 kB
Bounce:                0 kB
WritebackTmp:          0 kB
CommitLimit:    12358520 kB
Committed_AS:    5861400 kB
VmallocTotal:   34359738367 kB
VmallocUsed:           0 kB
VmallocChunk:          0 kB
HardwareCorrupted:     0 kB
AnonHugePages:    700416 kB
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].value, "Command result is insufficient")

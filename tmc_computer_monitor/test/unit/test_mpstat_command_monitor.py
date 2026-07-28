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


class TestMpstatCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 1: Specify all items and normal case (insert the actual command result)"""
        m = tmc_computer_monitor.MpstatCommandMonitor(
            "CPU",
            {
                "items": [
                    "User",
                    "Nice",
                    "Sys",
                    "IOwait",
                    "IRQ",
                    "SoftIRQ",
                    "Steal",
                    "Guest",
                    "Gnice",
                    "Idle",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Linux 3.14.38-rt36-tmc (hsrb)   08/23/17        _x86_64_        (8 CPU)

12:59:00 CPU  %usr %nice  %sys %iowait  %irq %soft %steal %guest %gnice   %idle
12:59:01 all  1.89  0.00  0.88    0.00  0.00  0.00   0.00   0.00   0.00   97.23
12:59:01   0  0.98  0.00  1.96    0.00  0.00  0.00   0.00   0.00   0.00   97.06
12:59:01   1  0.00  0.00  2.00    0.00  0.00  0.00   0.00   0.00   0.00   98.00
12:59:01   2  4.04  0.00  2.02    0.00  0.00  0.00   0.00   0.00   0.00   93.94
12:59:01   3  1.02  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00   98.98
12:59:01   4  0.00  0.00  0.98    0.00  0.00  0.98   0.00   0.00   0.00   98.04
12:59:01   5  2.08  0.00  1.04    0.00  0.00  0.00   0.00   0.00   0.00   96.88
12:59:01   6  0.00  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00  100.00
12:59:01   7  4.04  0.00  1.01    0.00  0.00  0.00   0.00   0.00   0.00   94.95

Average: CPU  %usr %nice  %sys %iowait  %irq %soft %steal %guest %gnice   %idle
Average: all  1.00  2.00  3.00    4.00  5.00  6.00   7.00   8.00   9.00   10.00
Average:   0  0.98  0.00  1.96    0.00  0.00  0.00   0.00   0.00   0.00   97.06
Average:   1  0.00  0.00  2.00    0.00  0.00  0.00   0.00   0.00   0.00   98.00
Average:   2  4.04  0.00  2.02    0.00  0.00  0.00   0.00   0.00   0.00   93.94
Average:   3  1.02  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00   98.98
Average:   4  0.00  0.00  0.98    0.00  0.00  0.98   0.00   0.00   0.00   98.04
Average:   5  2.08  0.00  1.04    0.00  0.00  0.00   0.00   0.00   0.00   96.88
Average:   6  0.00  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00  100.00
Average:   7  4.04  0.00  1.01    0.00  0.00  0.00   0.00   0.00   0.00   94.95
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "user=1.00/sys=3.00/io=4.00/idle=10.00")
        self.assertEqual(len(status.values), 9 * 10)
        # Check only the very first key
        self.assertEqual(status.values[0].key, "CPU all: User")
        self.assertEqual(status.values[0].value, "1.00")
        self.assertEqual(status.values[1].key, "CPU all: Nice")
        self.assertEqual(status.values[1].value, "2.00")
        self.assertEqual(status.values[2].key, "CPU all: Sys")
        self.assertEqual(status.values[2].value, "3.00")
        self.assertEqual(status.values[3].key, "CPU all: IOwait")
        self.assertEqual(status.values[3].value, "4.00")
        self.assertEqual(status.values[4].key, "CPU all: IRQ")
        self.assertEqual(status.values[4].value, "5.00")
        self.assertEqual(status.values[5].key, "CPU all: SoftIRQ")
        self.assertEqual(status.values[5].value, "6.00")
        self.assertEqual(status.values[6].key, "CPU all: Steal")
        self.assertEqual(status.values[6].value, "7.00")
        self.assertEqual(status.values[7].key, "CPU all: Guest")
        self.assertEqual(status.values[7].value, "8.00")
        self.assertEqual(status.values[8].key, "CPU all: Gnice")
        self.assertEqual(status.values[8].value, "9.00")
        self.assertEqual(status.values[9].key, "CPU all: Idle")
        self.assertEqual(status.values[9].value, "10.00")

    def test_ok_part_items(self):
        """TESTCASE 2: Specify some items and normal case"""
        m = tmc_computer_monitor.MpstatCommandMonitor(
            "CPU", {"items": ["User", "Nice", "Sys", "Idle"]}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Linux 3.14.38-rt36-tmc (hsrb)   08/23/17        _x86_64_        (8 CPU)

12:59:00 CPU  %usr %nice  %sys %iowait  %irq %soft %steal %guest %gnice   %idle
12:59:01 all  1.89  0.00  0.88    0.00  0.00  0.00   0.00   0.00   0.00   97.23
12:59:01   0  0.98  0.00  1.96    0.00  0.00  0.00   0.00   0.00   0.00   97.06
12:59:01   1  0.00  0.00  2.00    0.00  0.00  0.00   0.00   0.00   0.00   98.00
12:59:01   2  4.04  0.00  2.02    0.00  0.00  0.00   0.00   0.00   0.00   93.94
12:59:01   3  1.02  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00   98.98
12:59:01   4  0.00  0.00  0.98    0.00  0.00  0.98   0.00   0.00   0.00   98.04
12:59:01   5  2.08  0.00  1.04    0.00  0.00  0.00   0.00   0.00   0.00   96.88
12:59:01   6  0.00  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00  100.00
12:59:01   7  4.04  0.00  1.01    0.00  0.00  0.00   0.00   0.00   0.00   94.95

Average: CPU  %usr %nice  %sys %iowait  %irq %soft %steal %guest %gnice   %idle
Average: all  1.00  2.00  3.00    4.00  5.00  6.00   7.00   8.00   9.00   10.00
Average:   0  0.98  0.00  1.96    0.00  0.00  0.00   0.00   0.00   0.00   97.06
Average:   1  0.00  0.00  2.00    0.00  0.00  0.00   0.00   0.00   0.00   98.00
Average:   2  4.04  0.00  2.02    0.00  0.00  0.00   0.00   0.00   0.00   93.94
Average:   3  1.02  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00   98.98
Average:   4  0.00  0.00  0.98    0.00  0.00  0.98   0.00   0.00   0.00   98.04
Average:   5  2.08  0.00  1.04    0.00  0.00  0.00   0.00   0.00   0.00   96.88
Average:   6  0.00  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00  100.00
Average:   7  4.04  0.00  1.01    0.00  0.00  0.00   0.00   0.00   0.00   94.95
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "user=1.00/sys=3.00/io=4.00/idle=10.00")
        self.assertEqual(len(status.values), 9 * 4)

    def test_ng_invalid_config_item(self):
        """TESTCASE 3: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.MpstatCommandMonitor("CPU", {"items": ["dummy"]})
        self.assertEqual(str(ex.exception), "Unknown item: dummy in CPU")

    def test_ng_insufficient_result(self):
        """TESTCASE 4: Unable to parse the command execution result (insufficient result)"""
        m = tmc_computer_monitor.MpstatCommandMonitor("CPU", {"items": ["User"]})
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Linux 3.14.38-rt36-tmc (hsrb)   08/23/17        _x86_64_        (8 CPU)

12:59:00 CPU  %usr %nice  %sys %iowait  %irq %soft %steal %guest %gnice   %idle
12:59:01 all  1.89  0.00  0.88    0.00  0.00  0.00   0.00   0.00   0.00   97.23
12:59:01   0  0.98  0.00  1.96    0.00  0.00  0.00   0.00   0.00   0.00   97.06
12:59:01   1  0.00  0.00  2.00    0.00  0.00  0.00   0.00   0.00   0.00   98.00
12:59:01   2  4.04  0.00  2.02    0.00  0.00  0.00   0.00   0.00   0.00   93.94
12:59:01   3  1.02  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00   98.98
12:59:01   4  0.00  0.00  0.98    0.00  0.00  0.98   0.00   0.00   0.00   98.04
12:59:01   5  2.08  0.00  1.04    0.00  0.00  0.00   0.00   0.00   0.00   96.88
12:59:01   6  0.00  0.00  0.00    0.00  0.00  0.00   0.00   0.00   0.00  100.00
12:59:01   7  4.04  0.00  1.01    0.00  0.00  0.00   0.00   0.00   0.00   94.95
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "IndexError")
        self.assertEqual(status.values[1].value, "list index out of range")

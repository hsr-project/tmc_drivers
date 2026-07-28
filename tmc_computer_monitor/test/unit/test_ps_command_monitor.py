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


class TestPsCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 5: Specify all items and in the normal case (insert the actual command result)"""
        m = tmc_computer_monitor.PsCommandMonitor(
            "Process",
            {
                "sort_key": "UseCpuRate",
                "num_to_disp": 5,
                "items": [
                    "UserID",
                    "ProcessID",
                    "UseCpuRate",
                    "UseMemoryRate",
                    "UseVirtualMemorySize",
                    "UsePhisicalMemorySize",
                    "TypeOfTerminal",
                    "ProcessStatus",
                    "ProcessStartTime",
                    "ExecutionTime",
                    "ExecutionCommand",
                ],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root     729  1.0  10.0   4400  1264 ?     Ss   03:57   0:00 /usr/sbin/acpid
message+ 733  2.0  11.0  44288  5216 ?     Ss   03:57   0:02 /usr/bin/dbus
root     736  3.0  12.0      0     0 ?     S<   08:27   0:00 [kworker/6:1H]
avahi    774  4.0  13.0  44788   340 ?     S    03:57   0:00 avahi-daemon
root     816  5.0  9.1 274824  9424 ?     Ssl  03:57   0:00 /usr/sbin/cups
root     818  6.0  8.2 457788 19800 ?     Ssl  03:57   0:00 /usr/sbin/Network
root     869  7.0  7.1 292196  8252 ?     SLsl 03:57   0:00 /usr/sbin/lightdm
root     876  13.0  6.1 298708 13056 ?     Ssl  03:57   0:00 /usr/lib/policykit
root     927  12.0  5.0      0     0 ?     S    04:49   0:00 [kworker/4:2]
root     969  11.0  1.3 519792 112220 tty7 Ss+  03:57   2:32 /usr/lib/xorg/Xorg
root     976  10.0  0.0  65520  5644 ?     Ss   03:57   0:00 /usr/sbin/sshd -D
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Use CPU rate[%]=13.0 [/usr/lib/policykit]")
        self.assertEqual(len(status.values), 5 * 11)
        # Check only the very first key
        self.assertEqual(status.values[0].key, "Process 1 User ID")
        self.assertEqual(status.values[0].value, "root")
        self.assertEqual(status.values[1].key, "Process 1 Process ID")
        self.assertEqual(status.values[1].value, "876")
        self.assertEqual(status.values[2].key, "Process 1 Use CPU rate[%]")
        self.assertEqual(status.values[2].value, "13.0")
        self.assertEqual(status.values[3].key, "Process 1 Use memory rate[%]")
        self.assertEqual(status.values[3].value, "6.1")
        self.assertEqual(status.values[4].key, "Process 1 Use virtual memory size[kb]")
        self.assertEqual(status.values[4].value, "298708")
        self.assertEqual(status.values[5].key, "Process 1 Use phisical memory size[kb]")
        self.assertEqual(status.values[5].value, "13056")
        self.assertEqual(status.values[6].key, "Process 1 Type of terminal")
        self.assertEqual(status.values[6].value, "?")
        self.assertEqual(status.values[7].key, "Process 1 Process status")
        self.assertEqual(status.values[7].value, "Ssl")
        self.assertEqual(status.values[8].key, "Process 1 process start time")
        self.assertEqual(status.values[8].value, "03:57")
        self.assertEqual(status.values[9].key, "Process 1 Execution time")
        self.assertEqual(status.values[9].value, "0:00")
        self.assertEqual(status.values[10].key, "Process 1 Execution command")
        self.assertEqual(status.values[10].value, "/usr/lib/policykit")

    def test_ok_part_items(self):
        """TESTCASE 6: Specify some items and in the normal case"""
        m = tmc_computer_monitor.PsCommandMonitor(
            "Process",
            {
                "sort_key": "UseMemoryRate",
                "num_to_disp": 3,
                "items": ["UserID", "ProcessID", "ExecutionCommand"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root     729  1.0  10.0   4400  1264 ?     Ss   03:57   0:00 /usr/sbin/acpid
message+ 733  2.0  11.0  44288  5216 ?     Ss   03:57   0:02 /usr/bin/dbus
root     736  3.0  12.0      0     0 ?     S<   08:27   0:00 [kworker/6:1H]
avahi    774  4.0  13.0  44788   340 ?     S    03:57   0:00 avahi-daemon
root     816  5.0  9.1 274824  9424 ?     Ssl  03:57   0:00 /usr/sbin/cups
root     818  6.0  8.2 457788 19800 ?     Ssl  03:57   0:00 /usr/sbin/Network
root     869  7.0  7.1 292196  8252 ?     SLsl 03:57   0:00 /usr/sbin/lightdm
root     876  13.0  6.1 298708 13056 ?     Ssl  03:57   0:00 /usr/lib/policykit
root     927  12.0  5.0      0     0 ?     S    04:49   0:00 [kworker/4:2]
root     969  11.0  1.3 519792 112220 tty7 Ss+  03:57   2:32 /usr/lib/xorg/Xorg
root     976  10.0  0.0  65520  5644 ?     Ss   03:57   0:00 /usr/sbin/sshd -D
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Use memory rate[%]=13.0 [avahi-daemon]")

        self.assertEqual(len(status.values), 3 * 3)

    def test_ng_invalid_config_item(self):
        """TESTCASE 7: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.PsCommandMonitor(
                "Process",
                {"sort_key": "UseMemoryRate", "num_to_disp": 3, "items": ["dummy"]},
            )
        self.assertEqual(str(ex.exception), "Unknown item: dummy in Process")

    def test_ng_invalid_config_name(self):
        """TESTCASE 8: Invalid item name"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.PsCommandMonitor(
                "Process",
                {
                    "sort_key": "UseMemoryRate",
                    "dummy": 3,
                    "items": ["UserID", "ProcessID", "ExecutionCommand"],
                },
            )
        self.assertEqual(str(ex.exception), "num_to_disp is not defined in config file")

    def test_ng_insufficient_result(self):
        """TESTCASE 9: Unable to parse the command execution result (missing command part)"""
        m = tmc_computer_monitor.PsCommandMonitor(
            "Process",
            {
                "sort_key": "UseMemoryRate",
                "num_to_disp": 3,
                "items": ["UserID", "ProcessID", "ExecutionCommand"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root       729  1.0  10.0   4400  1264 ?        Ss   03:57   0:00 /usr/sbin/acp
message+   733  2.0  11.0  44288  5216 ?        Ss   03:57   0:02
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "IndexError")
        self.assertEqual(status.values[1].value, "list index out of range")

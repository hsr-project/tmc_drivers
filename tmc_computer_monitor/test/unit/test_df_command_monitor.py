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


class TestDfCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 18: Specify all items, normal case (insert the actual command result)"""
        m = tmc_computer_monitor.DfCommandMonitor(
            "Disk",
            {"target": ["/"], "items": ["Filesystem", "Size", "Used", "Avail", "Use%"]},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Filesystem     1K-blocks      Used Available Use% Mounted on
udev            16417400         0  16417400   0% /dev
tmpfs            3287176     10008   3277168   1% /run
/dev/sda2      229049588 152088116  65303336  70% /
tmpfs           16435872       176  16435696   1% /dev/shm
tmpfs               5120         4      5116   1% /run/lock
tmpfs           16435872         0  16435872   0% /sys/fs/cgroup
/dev/sda1         523248      3632    519616   1% /boot/efi
tmpfs            3287176        44   3287132   1% /run/user/1000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "/: Used 70%")
        self.assertEqual(len(status.values), 40)
        # Check only the very first key
        self.assertEqual(status.values[0].key, "/dev: Filesystem")
        self.assertEqual(status.values[0].value, "udev")
        self.assertEqual(status.values[1].key, "/dev: Size")
        self.assertEqual(status.values[1].value, "16417400")
        self.assertEqual(status.values[2].key, "/dev: Used")
        self.assertEqual(status.values[2].value, "0")
        self.assertEqual(status.values[3].key, "/dev: Avail")
        self.assertEqual(status.values[3].value, "16417400")
        self.assertEqual(status.values[4].key, "/dev: Use%")
        self.assertEqual(status.values[4].value, "0%")

    def test_ok_part_items(self):
        """TESTCASE 19: Specify some items, normal case"""
        m = tmc_computer_monitor.DfCommandMonitor(
            "Disk", {"target": ["/run"], "items": ["Filesystem", "Use%"]}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Filesystem     1K-blocks      Used Available Use% Mounted on
udev            16417400         0  16417400   0% /dev
tmpfs            3287176     10008   3277168   1% /run
/dev/sda2      229049588 152088116  65303336  70% /
tmpfs           16435872       176  16435696   1% /dev/shm
tmpfs               5120         4      5116   1% /run/lock
tmpfs           16435872         0  16435872   0% /sys/fs/cgroup
/dev/sda1         523248      3632    519616   1% /boot/efi
tmpfs            3287176        44   3287132   1% /run/user/1000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "/run: Used 1%")
        self.assertEqual(len(status.values), 16)

    def test_ok_multi_targets_all(self):
        """TESTCASE 20: Specify multiple targets, all match (display the result of the first matching target on the diagram)"""
        m = tmc_computer_monitor.DfCommandMonitor(
            "Disk",
            {
                "target": ["/", "/run", "/run/user/1000"],
                "items": ["Filesystem", "Use%"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Filesystem     1K-blocks      Used Available Use% Mounted on
udev            16417400         0  16417400   0% /dev
tmpfs            3287176     10008   3277168   1% /run
/dev/sda2      229049588 152088116  65303336  70% /
tmpfs           16435872       176  16435696   1% /dev/shm
tmpfs               5120         4      5116   1% /run/lock
tmpfs           16435872         0  16435872   0% /sys/fs/cgroup
/dev/sda1         523248      3632    519616   1% /boot/efi
tmpfs            3287176        44   3287132   1% /run/user/1000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "/run: Used 1%")
        self.assertEqual(len(status.values), 16)

    def test_ok_multi_targets_part(self):
        """TESTCASE 21: Specify multiple targets, partial match (display the result of the first matching target on the diagram)"""
        m = tmc_computer_monitor.DfCommandMonitor(
            "Disk",
            {
                "target": ["/", "/run/lock", "/dymmy"],
                "items": ["Filesystem", "Size", "Use%"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Filesystem     1K-blocks      Used Available Use% Mounted on
udev            16417400         0  16417400   0% /dev
tmpfs            3287176     10008   3277168   1% /run
/dev/sda2      229049588 152088116  65303336  70% /
tmpfs           16435872       176  16435696   1% /dev/shm
tmpfs               5120         4      5116   1% /run/lock
tmpfs           16435872         0  16435872   0% /sys/fs/cgroup
/dev/sda1         523248      3632    519616   1% /boot/efi
tmpfs            3287176        44   3287132   1% /run/user/1000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "/: Used 70%")
        self.assertEqual(len(status.values), 24)

    def test_ng_invalid_config_item(self):
        """TESTCASE 22: No target specified"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.DfCommandMonitor("Disk", {"items": ["Filesystem"]})
        self.assertEqual(str(ex.exception), "target is not defined in config file")

    def test_ng_target_is_not_list(self):
        """TESTCASE 23: Target is not a list"""
        with self.assertRaises(ValueError) as ex:
            tmc_computer_monitor.DfCommandMonitor(
                "Disk", {"target": "/", "items": ["Filesystem"]}
            )
        self.assertEqual(str(ex.exception), "target of Disk is not list")

    def test_ok_no_target_disk(self):
        """TESTCASE 24: Command execution result cannot be parsed (result is insufficient)"""
        m = tmc_computer_monitor.DfCommandMonitor(
            "Disk",
            {
                "target": ["/", "/run"],
                "items": ["Filesystem", "Size", "Used", "Avail", "Use%"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Filesystem     1K-blocks      Used Available Use% Mounted on
udev            16417400         0  16417400   0% /dev
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "/, /run not found")
        self.assertEqual(len(status.values), 5)

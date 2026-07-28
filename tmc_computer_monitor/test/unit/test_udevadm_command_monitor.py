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
from tmc_computer_monitor import UdevAdmCommandMonitor


class TestUdevAdmCommandMonitor(unittest.TestCase):
    def test_ok_command(self):
        """TESTCASE 62"""
        m = UdevAdmCommandMonitor(
            "Device1", {"device": "/dev/test", "hardware_id": "Device01"}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""P: /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.1/1-1.1:1.0/usbmisc/lp0
N: usb/lp0
E: DEVNAME=/dev/usb/lp0
E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/\
    1-1/1-1.1/1-1.1:1.0/usbmisc/lp0
E: MAJOR=180
E: MINOR=0
E: SUBSYSTEM=usbmisc
E: SYSTEMD_ALIAS=/sys/devices/pci/selphy/cp1300
E: SYSTEMD_WANTS=cupsd.service
E: TAGS=:systemd:
E: USEC_INITIALIZED=639945600
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "OK")
        self.assertEqual(status.hardware_id, "Device01")

    def test_ng_command(self):
        """TESTCASE 63"""
        result = (
            "Unknown device, --name=, --path=, or "
            "absolute path in /dev/ or /sys expected."
        )
        m = UdevAdmCommandMonitor(
            "Device1", {"device": "/dev/test", "hardware_id": "Device01"}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = result
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "failed to update diagnostic for Device1")
        self.assertEqual(status.hardware_id, "Device01")
        self.assertEqual(status.values[0].key, "exception")
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].key, "message")
        self.assertEqual(status.values[1].value, "Unknown device.")
        self.assertEqual(status.values[2].key, "command_result")
        self.assertEqual(status.values[2].value, result)

    def test_ng_no_device(self):
        """TESTCASE 64"""
        result = (
            "A device name or path is required"
        )
        m = UdevAdmCommandMonitor(
            "Device1", {}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = result
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "failed to update diagnostic for Device1")
        self.assertEqual(status.values[0].key, "exception")
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].key, "message")
        self.assertEqual(status.values[1].value, "A device name or path is required")

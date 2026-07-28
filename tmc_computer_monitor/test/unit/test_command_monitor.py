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

import sys
import unittest

from diagnostic_msgs.msg import DiagnosticStatus
import tmc_computer_monitor


class TestCommandMonitor(unittest.TestCase):
    """Test of the common processing part"""

    def test_ng_command_not_exists(self):
        """TESTCASE 50: Unable to execute due to missing command"""
        m = tmc_computer_monitor.MpstatCommandMonitor(
            "CPU", {"items": ["User", "Nice", "Sys", "Idle"]}
        )
        m._command = ["/unknown_command"]
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        if sys.version_info.major == 3:
            self.assertEqual(status.values[0].value, "FileNotFoundError")
        else:
            self.assertEqual(status.values[0].value, "OSError")

    def test_ng_command_fails(self):
        """TESTCASE 51: Command did not terminate successfully"""
        m = tmc_computer_monitor.MpstatCommandMonitor(
            "CPU", {"publish_rate": 1.0, "items": ["User", "Nice", "Sys", "Idle"]}
        )
        m._command = ["/bin/false"]
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "RuntimeError")

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

import os # noqa: E261, F401, this is used in mock.patch
import unittest
import unittest.mock as mock

from diagnostic_msgs.msg import DiagnosticStatus
import tmc_computer_monitor


class TestLsusbCommandMonitor(unittest.TestCase):

    def test_ok_full_items(self):
        """TESTCASE 34: Specify all items and in the normal case (insert the actual command result)"""
        m = tmc_computer_monitor.LsusbCommandMonitor("USB", {})
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 003: ID 0a12:0001 Cambridge Silicon Radio, Ltd Bluetooth Dongle
Bus 003 Device 002: ID 2109:2812 VIA Labs, Inc. VL812 Hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 004: ID 1e10:3300 Point Grey Research, Inc.
Bus 002 Device 003: ID 1e10:3300 Point Grey Research, Inc.
Bus 002 Device 002: ID 13e6:1210 TechnoScope Co., Ltd.
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 006: ID 064f:2af9 WIBU-Systems AG
Bus 001 Device 011: ID 1415:2000 Nam Tai Product Ltd. Inc. Sony Playstation Eye
Bus 001 Device 012: ID 1d27:0601 ASUS
Bus 001 Device 004: ID 13e6:1212 TechnoScope Co., Ltd.
Bus 001 Device 007: ID 10c4:ea60 CP210x UART Bridge / myAVR mySmartUSB light
Bus 001 Device 005: ID 1742:1302
Bus 001 Device 003: ID 13e6:1212 TechnoScope Co., Ltd.
Bus 001 Device 002: ID 17ef:6047 Lenovo
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "17 devices connected")
        self.assertEqual(len(status.values), 17)
        # Check the first and last keys
        self.assertEqual(status.values[0].key, "004/001")
        self.assertEqual(
            status.values[0].value, "ID 1d6b:0003 Linux Foundation 3.0 root hub"
        )
        self.assertEqual(status.values[16].key, "001/001")
        self.assertEqual(
            status.values[16].value, "ID 1d6b:0002 Linux Foundation 2.0 root hub"
        )

    def test_ng_empty(self):
        """TESTCASE 35: The command execution result cannot be parsed (empty)"""
        m = tmc_computer_monitor.LsusbCommandMonitor("USB", {})
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].value, "No device is connected")

    def test_ng_invalid_config_item(self):
        """TESTCASE 36: When unnecessary items are included"""
        with self.assertRaises(AttributeError) as ex:
            tmc_computer_monitor.LsusbCommandMonitor("USB", {"items": ["dummy"]})
        self.assertEqual(
            str(ex.exception),
            "'LsusbCommandMonitor' object has no attribute '_item_info'",
        )

    def test_ok_pid_vid(self):
        """TESTCASE 35"""
        pid = "046d"
        vid = "c07e"

        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB", {"device": f"{pid}:{vid}", "hardware_id": "Mouse"}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = (
            r"""Bus 001 Device 003: ID 046d:c07e Logitech, Inc. G402 Gaming Mouse"""
        )
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "OK")
        self.assertEqual(status.hardware_id, "Mouse")

    def test_ok_error_pid_vid(self):
        """TESTCASE 36"""
        pid = "046d"
        vid = "c07e"
        config = {"device": f"{pid}:{vid}", "hardware_id": "Mouse"}
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB", config
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = " "

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "failed to update diagnostic for USB")
        self.assertEqual(status.hardware_id, "Mouse")
        self.assertEqual(status.values[0].key, "exception")
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].key, "message")
        self.assertEqual(status.values[1].value, "Device not found.")
        self.assertEqual(status.values[2].key, "command_result")
        self.assertEqual(status.values[2].value, " ")

    def test_ok_one_keyword(self):
        """TESTCASE 37"""
        pid = "046d"
        vid = "c07e"
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {
                "device": f"{pid}:{vid}",
                "grep_match": ["iSerial"],
                "hardware_id": "Mouse",
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Couldn't open device, some information will be missing
    iSerial                 3 497846473935"""

        status = m.get_diag().status[0]
        self.assertEqual(status.message, "OK")
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.hardware_id, "Mouse")

    def test_ok_error_one_keyword(self):
        """TESTCASE 38"""
        pid = "046d"
        vid = "c07e"
        keyword = ["iSerial"]
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {"device": f"{pid}:{vid}", "grep_match": keyword, "hardware_id": "Mouse"},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = " "
        status = m.get_diag().status[0]
        self.assertEqual(status.message, "failed to update diagnostic for USB")
        self.assertEqual(status.hardware_id, "Mouse")
        self.assertEqual(status.values[0].key, "exception")
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].key, "message")
        self.assertEqual(status.values[1].value, "Device not found.")
        self.assertEqual(status.values[2].key, "command_result")
        self.assertEqual(status.values[2].value, " ")

    def test_ok_two_keywords(self):
        """TESTCASE 39"""
        pid = "046d"
        vid = "c07e"
        keyword = ["iSerial", "497846473935"]
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {"device": f"{pid}:{vid}", "grep_match": keyword, "hardware_id": "Mouse"},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Couldn't open device, some information will be missing
    iSerial                 3 497846473935"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "OK")
        self.assertEqual(status.hardware_id, "Mouse")

    def test_ok_error_two_keywords(self):
        """TESTCASE 40"""
        pid = "046d"
        vid = "c07e"
        keyword = ["iSerial", "497846473935"]
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {"device": f"{pid}:{vid}", "grep_match": keyword, "hardware_id": "Mouse"},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = " "

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "failed to update diagnostic for USB")
        self.assertEqual(status.hardware_id, "Mouse")
        self.assertEqual(status.values[0].key, "exception")
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(status.values[1].key, "message")
        self.assertEqual(status.values[1].value, "Device not found.")
        self.assertEqual(status.values[2].key, "command_result")
        self.assertEqual(status.values[2].value, " ")

    @mock.patch("os.getenv")
    def test_ok_keyword_env(self, mock_getenv):
        """TESTCASE 41"""
        mock_getenv.return_value = "497846473935"
        pid = "046d"
        vid = "c07e"
        keyword = ["iSerial", "${SERIAL_NUMBER}"]
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {"device": f"{pid}:{vid}", "grep_match": keyword, "hardware_id": "Mouse"},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Couldn't open device, some information will be missing
    iSerial                 3 497846473935"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "OK")
        self.assertEqual(status.hardware_id, "Mouse")

    @mock.patch("os.getenv")
    def test_ok_err_keyword_env(self, mock_getenv):
        """TESTCASE 41"""
        mock_getenv.return_value = "1111111111111111"
        pid = "046d"
        vid = "c07e"
        keyword = ["iSerial", "${SERIAL_NUMBER}"]
        m = tmc_computer_monitor.LsusbCommandMonitor(
            "USB",
            {"device": f"{pid}:{vid}", "grep_match": keyword, "hardware_id": "Mouse"},
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = " "

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "failed to update diagnostic for USB")
        self.assertEqual(status.hardware_id, "Mouse")

    def test_ng_pid_vid(self):
        """TESTCASE 45"""
        pid = "zzzz"
        vid = "xxxx"
        try:
            m = tmc_computer_monitor.LsusbCommandMonitor(
                "USB", {"device": f"{pid}:{vid}", "hardware_id": "Mouse"}
            )
            m._execute_command = mock.Mock()
            m._execute_command.return_value = " "
            m.get_diag().status[0]
            return False
        except Exception as err:
            self.assertEqual(str(err), "PID:VID value is invalid. PID:VID must be hexadecimales values")

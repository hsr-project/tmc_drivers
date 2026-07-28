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
from tmc_computer_monitor import FileMonitor


class TestFileMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 43: In the case of normal operation (include the result of file output)"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111", "- id: 22222222"],
                            "notcontain": ["- id: 333333333", "dummy"],
                        }
                    ],
                    "DummyConfig": [
                        {
                            "path": "test/dummy.txt",
                            "contain": ["test"],
                            "notcontain": ["dummy"],
                        }
                    ],
                }
            },
        )

        m._file_open = mock.Mock()
        m._file_open.side_effect = [
            ["cameras:\n", "- id: 11111111\n", "master: true\n", "- id: 22222222\n"],
            ["test\n", "hoge\n"],
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.OK)
        self.assertEqual(status[0].message, "OK")
        self.assertEqual(len(status[0].values), 1)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")
        self.assertEqual(status[1].level, DiagnosticStatus.OK)
        self.assertEqual(status[1].message, "OK")
        self.assertEqual(len(status[1].values), 1)
        self.assertEqual(status[1].values[0].key, "Path")
        self.assertEqual(status[1].values[0].value, "test/dummy.txt")

    def test_ok_path_only(self):
        """TESTCASE 44: When only the path is set"""
        m = FileMonitor(
            "File",
            {"files": {"CameraConfig": [{"path": "/opt/test/camera_setting.yml"}]}},
        )
        m._file_open = mock.Mock()
        m._file_open.return_value = [
            "cameras:\n",
            "- id: 11111111\n",
            "master: true\n",
            "- id: 33333333\n",
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.OK)
        self.assertEqual(len(status[0].values), 1)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")

    def test_ng_file_not_exist(self):
        """TESTCASE 45: When the target file does not exist"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111", "- id: 22222222"],
                            "notcontain": ["- id: 333333333", "dummy"],
                            "error_message": "Error",
                        },
                    ]
                }
            },
        )
        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.ERROR)
        # Even if error_message is set, the message outputs a fixed format
        self.assertEqual(
            status[0].message, "No such file: /opt/test/camera_setting.yml"
        )
        self.assertEqual(len(status[0].values), 1)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")

    def test_ng_contain(self):
        """TESTCASE 46: When the expected content is not included"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111", "- id: 22222222"],
                            "notcontain": ["- id: 33333333"],
                        }
                    ],
                    "DummyConfig": [{"path": "test/dummy.txt", "contain": ["test"]}],
                }
            },
        )
        m._file_open = mock.Mock()
        m._file_open.side_effect = [
            ["cameras:\n", "- id: 00000000\n", "master: true\n", "- id: 22222222\n"],
            ["test\n", "hoge\n"],
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(status[0].message, "File does not contain a necessary line")
        self.assertEqual(len(status[0].values), 2)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")
        self.assertEqual(status[0].values[1].key, "Contain")
        self.assertEqual(status[0].values[1].value, "['- id: 11111111'] not contain")
        self.assertEqual(status[1].level, DiagnosticStatus.OK)
        self.assertEqual(status[1].message, "OK")
        self.assertEqual(len(status[1].values), 1)
        self.assertEqual(status[1].values[0].key, "Path")
        self.assertEqual(status[1].values[0].value, "test/dummy.txt")

    def test_ng_contain_set_error_message(self):
        """TESTCASE 47: When the expected content is not included (error message specified)"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111", "- id: 22222222"],
                            "notcontain": ["- id: 33333333"],
                            "error_message": "Configuration error.",
                        }
                    ]
                }
            },
        )
        m._file_open = mock.Mock()
        m._file_open.return_value = [
            "cameras:\n",
            "- id: 00000000\n",
            "master: true\n",
            "- id: 22222222\n",
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(status[0].message, "Configuration error.")
        self.assertEqual(len(status[0].values), 2)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")
        self.assertEqual(status[0].values[1].key, "Contain")
        self.assertEqual(status[0].values[1].value, "['- id: 11111111'] not contain")

    def test_ng_notcontain(self):
        """TESTCASE 48: When unexpected content is included"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111"],
                            "notcontain": ["- id: 22222222", "- id: 33333333"],
                        }
                    ]
                }
            },
        )
        m._file_open = mock.Mock()
        m._file_open.return_value = [
            "cameras:\n",
            "- id: 11111111\n",
            "master: true\n",
            "- id: 33333333\n",
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(status[0].message, "File contains an invalid content")
        self.assertEqual(len(status[0].values), 2)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")
        self.assertEqual(status[0].values[1].key, "NotContain")
        self.assertEqual(status[0].values[1].value, "'- id: 33333333' in line:4")

    def test_ng_notcontain_set_error_message(self):
        """TESTCASE 49: When unexpected content is included (error message specified)"""
        m = FileMonitor(
            "File",
            {
                "files": {
                    "CameraConfig": [
                        {
                            "path": "/opt/test/camera_setting.yml",
                            "contain": ["- id: 11111111"],
                            "notcontain": ["- id: 22222222", "- id: 33333333"],
                            "error_message": "Configuration error.",
                        }
                    ]
                }
            },
        )
        m._file_open = mock.Mock()
        m._file_open.return_value = [
            "cameras:\n",
            "- id: 11111111\n",
            "master: true\n",
            "- id: 33333333\n",
        ]

        status = m.get_diag().status
        self.assertEqual(status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(status[0].message, "Configuration error.")
        self.assertEqual(len(status[0].values), 2)
        self.assertEqual(status[0].values[0].key, "Path")
        self.assertEqual(status[0].values[0].value, "/opt/test/camera_setting.yml")
        self.assertEqual(status[0].values[1].key, "NotContain")
        self.assertEqual(status[0].values[1].value, "'- id: 33333333' in line:4")

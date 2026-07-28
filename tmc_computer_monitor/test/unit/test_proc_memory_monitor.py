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

import unittest
import unittest.mock as patch

from diagnostic_msgs.msg import DiagnosticStatus
import tmc_computer_monitor


class TestProcMemoryMonitor(unittest.TestCase):

    @patch.patch('builtins.open')
    @patch.patch('tmc_computer_monitor.proc_memory_monitor.os.path.exists')
    @patch.patch('tmc_computer_monitor.proc_memory_monitor.os.listdir')
    def test_full_data(self, mock_listdir, mock_exists, mock_open):
        mock_listdir.return_value = ['1', '2', '3', 'abc']
        mock_exists.return_value = True

        def mock_file_open(file, mode='r'):
            file_data = {
                '/proc/1/status': 'Name: proc1\nVmRSS: 1500 kB\nVmLck: 300 kB\n',
                '/proc/2/status': 'Name: proc2\nVmRSS: 2500 kB\nLocked: 500 kB\n',
                '/proc/3/status': 'Name: proc3\nVmRSS: 1000 kB\nVmLck: 200 kB\n',
            }
            return unittest.mock.mock_open(read_data=file_data[file]).return_value

        mock_open.side_effect = mock_file_open

        config = {'top_n': 2}
        monitor = tmc_computer_monitor.ProcMemoryMonitor('proc_memory', config, 'test_prefix/')

        diag = monitor.get_diag()

        self.assertEqual(len(diag.status), 1)
        status = diag.status[0]
        self.assertEqual(status.name, 'test_prefix/computer/proc_memory')
        self.assertEqual(status.hardware_id, os.uname()[1])
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, 'Top 2 processes by RSS and locked memory usage')
        self.assertEqual(len(status.values), 4)

        rss_values = [value for value in status.values if value.key.startswith('RSS Rank')]
        self.assertEqual(len(rss_values), 2)
        self.assertEqual(rss_values[0].value, 'PID 2  RSS 2500 kB  proc2')
        self.assertEqual(rss_values[1].value, 'PID 1  RSS 1500 kB  proc1')

        locked_values = [value for value in status.values if value.key.startswith('Locked Rank')]
        self.assertEqual(len(locked_values), 2)
        self.assertEqual(locked_values[0].value, 'PID 2  Locked 500 kB  proc2')
        self.assertEqual(locked_values[1].value, 'PID 1  Locked 300 kB  proc1')

    @patch.patch('builtins.open')
    @patch.patch('tmc_computer_monitor.proc_memory_monitor.os.path.exists')
    @patch.patch('tmc_computer_monitor.proc_memory_monitor.os.listdir')
    def test_locked_memory_missing_process(self, mock_listdir, mock_exists, mock_open):
        mock_listdir.return_value = ['1', '2']
        mock_exists.return_value = True

        def mock_file_open(file, mode='r'):
            file_data = {
                '/proc/1/status': 'Name: proc1\nVmRSS: 1500 kB\n',
                '/proc/2/status': 'Name: proc2\nVmRSS: 2500 kB\nLocked: 500 kB\n',
            }
            return unittest.mock.mock_open(read_data=file_data[file]).return_value

        mock_open.side_effect = mock_file_open

        config = {'top_n': 2}
        monitor = tmc_computer_monitor.ProcMemoryMonitor('proc_memory', config, 'test_prefix/')

        diag = monitor.get_diag()

        self.assertEqual(len(diag.status), 1)
        status = diag.status[0]
        self.assertEqual(status.name, 'test_prefix/computer/proc_memory')
        self.assertEqual(status.hardware_id, os.uname()[1])
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, 'Top 2 processes by RSS and locked memory usage')
        self.assertEqual(len(status.values), 4)

        rss_values = [value for value in status.values if value.key.startswith('RSS Rank')]
        self.assertEqual(len(rss_values), 2)
        self.assertEqual(rss_values[0].value, 'PID 2  RSS 2500 kB  proc2')
        self.assertEqual(rss_values[1].value, 'PID 1  RSS 1500 kB  proc1')

        locked_values = [value for value in status.values if value.key.startswith('Locked Rank')]
        self.assertEqual(len(locked_values), 2)
        self.assertEqual(locked_values[0].value, 'PID 2  Locked 500 kB  proc2')
        self.assertEqual(locked_values[1].value, 'No locked memory')

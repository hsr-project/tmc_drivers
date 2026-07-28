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
import unittest.mock as mock

from diagnostic_msgs.msg import DiagnosticStatus
import tmc_computer_monitor


def IsKeyValueIncluded(key_values, key, value):
    for key_value in key_values:
        if key_value.key == key and key_value.value == value:
            return True
    return False


class TestSensorsCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 14: Specify all items and test the normal case (insert the actual command result)"""
        m = tmc_computer_monitor.SensorsCommandMonitor(
            "Sensors", {"items": ["temp", "in", "fan"]}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""acpitz-virtual-0
temp1:
  temp1_input: 29.000
  temp1_crit: 119.000
temp2:
  temp2_input: 25.000
  temp2_crit: 119.000
temp10:
  temp10_input: 26.000
  temp10_crit: 119.000

coretemp-isa-0000
Physical id 0:
  temp1_input: 26.000
  temp1_max: 100.000
  temp1_crit: 100.000
  temp1_crit_alarm: 0.000
Core 0:
  temp2_input: 22.000
  temp2_max: 100.000
  temp2_crit: 100.000
  temp2_crit_alarm: 0.000
Core 1:
  temp3_input: 23.000
  temp3_max: 100.000
  temp3_crit: 100.000
  temp3_crit_alarm: 0.000
Core 2:
  temp4_input: 24.000
  temp4_max: 100.000
  temp4_crit: 100.000
  temp4_crit_alarm: 0.000
Core 3:
  temp5_input: 23.000
  temp5_max: 100.000
  temp5_crit: 100.000
  temp5_crit_alarm: 0.000

nct7802-i2c-6-2e
in0:
  in0_input: 3.348
  in0_min: 0.000
  in0_max: 4.092
  in0_alarm: 0.000
  in0_beep: 0.000
in1:
  in1_input: 1.836
fan1:
  fan1_input: 0.000
  fan1_min: 0.000
  fan1_alarm: 0.000
  fan1_beep: 0.000
fan2:
  fan2_input: 3301.000
  fan2_min: 0.000
  fan2_alarm: 0.000
  fan2_beep: 0.000
temp1:
  temp1_input: 127.875
  temp1_max: 85.000
  temp1_min: 0.000
  temp1_crit: 100.000
  temp1_max_alarm: 0.000
  temp1_min_alarm: 0.000
  temp1_crit_alarm: 0.000
  temp1_fault: 1.000
  temp1_type: 3.000
  temp1_beep: 0.000
temp2:
  temp2_input: 0.000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Temp: 29.0 °C/ Fan speed: 3301.0 rpm")
        self.assertEqual(len(status.values), 51)
        # Checking all 51 items is tough, so only a subset is verified
        self.assertTrue(
            IsKeyValueIncluded(status.values, "nct7802-i2c-6-2e/in0/beep", "0.0 V")
        )
        self.assertTrue(
            IsKeyValueIncluded(status.values, "nct7802-i2c-6-2e/in0/input", "3.348 V")
        )
        self.assertTrue(
            IsKeyValueIncluded(status.values, "nct7802-i2c-6-2e/in0/max", "4.092 V")
        )
        self.assertTrue(
            IsKeyValueIncluded(
                status.values, "coretemp-isa-0000/temp5/input", "23.0 °C"
            )
        )
        self.assertTrue(
            IsKeyValueIncluded(
                status.values, "coretemp-isa-0000/temp5/crit_alarm", "0.0 °C"
            )
        )
        self.assertTrue(
            IsKeyValueIncluded(status.values, "coretemp-isa-0000/temp5/max", "100.0 °C")
        )

    def test_ok_part_items(self):
        """TESTCASE 15: Specify some items and test the normal case"""
        m = tmc_computer_monitor.SensorsCommandMonitor(
            "Sensors", {"items": ["temp", "in"]}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""acpitz-virtual-0
temp1:
  temp1_input: 29.000
  temp1_crit: 119.000
temp2:
  temp2_input: 25.000
  temp2_crit: 119.000

coretemp-isa-0000
Physical id 0:
  temp1_input: 26.000
  temp1_max: 100.000
  temp1_crit: 100.000
  temp1_crit_alarm: 0.000
Core 0:
  temp2_input: 22.000
  temp2_max: 100.000
  temp2_crit: 100.000
  temp2_crit_alarm: 0.000
Core 1:
  temp3_input: 23.000
  temp3_max: 100.000
  temp3_crit: 100.000
  temp3_crit_alarm: 0.000
Core 2:
  temp4_input: 24.000
  temp4_max: 100.000
  temp4_crit: 100.000
  temp4_crit_alarm: 0.000
Core 3:
  temp5_input: 23.000
  temp5_max: 100.000
  temp5_crit: 100.000
  temp5_crit_alarm: 0.000

nct7802-i2c-6-2e
in0:
  in0_input: 3.348
  in0_min: 0.000
  in0_max: 4.092
  in0_alarm: 0.000
  in0_beep: 0.000
in1:
  in1_input: 1.836
fan1:
  fan1_input: 0.000
  fan1_min: 0.000
  fan1_alarm: 0.000
  fan1_beep: 0.000
fan2:
  fan2_input: 3301.000
  fan2_min: 0.000
  fan2_alarm: 0.000
  fan2_beep: 0.000
temp1:
  temp1_input: 127.875
  temp1_max: 85.000
  temp1_min: 0.000
  temp1_crit: 100.000
  temp1_max_alarm: 0.000
  temp1_min_alarm: 0.000
  temp1_crit_alarm: 0.000
  temp1_fault: 1.000
  temp1_type: 3.000
  temp1_beep: 0.000
temp2:
  temp2_input: 0.000
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Temp: 29.0 °C/ Fan speed: 3301.0 rpm")
        self.assertEqual(len(status.values), 41)

    def test_ng_invalid_config_item(self):
        """TESTCASE 16: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.SensorsCommandMonitor("Sensors", {"items": ["dummy"]})
        self.assertEqual(str(ex.exception), "Unknown item: dummy in Sensors")

    def test_ng_invalid_number(self):
        """TESTCASE 17: Command execution result cannot be parsed (contains non-numeric values)"""
        m = tmc_computer_monitor.SensorsCommandMonitor(
            "Sensors", {"items": ["in", "fan"]}
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = """acpitz-virtual-0:
  temp1_input: error
  temp1_max: 100.000
  temp1_crit: 100.000
  temp1_crit_alarm: 0.000
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "ValueError")
        if sys.version_info.major == 3:
            self.assertEqual(
                status.values[1].value, "could not convert string to float: ' error'"
            )
        else:
            self.assertEqual(
                status.values[1].value, "could not convert string to float: error"
            )

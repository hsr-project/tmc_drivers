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


class TestNetstatCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 25: Specify all items, normal case (insert actual command results)"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["enp0s31f6"],
                "items": [
                    "Interface",
                    "MTU",
                    "ReceiveOK",
                    "ReceiveERR",
                    "ReceiveDRP",
                    "ReceiveOVR",
                    "SendOK",
                    "SendERR",
                    "SendDRP",
                    "SendOVR",
                    "Flg",
                ],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface      MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0   1500         0      0      0 0             0    0    0    0 BMU
enp0s31f6 1500    189108      0 175593 0           404    0    0    0 BMRU
ens15     1500    445362      0      0 0        223953    0    0    0 BMRU
lo       65536  68822265      0      0 0      68822265    0    0    0 LRU

"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "enp0s31f6: RXOK/189108 TXOK/404")
        self.assertEqual(len(status.values), 44)
        # Check only the very first key
        self.assertEqual(status.values[0].key, "docker0: Interface")
        self.assertEqual(status.values[0].value, "docker0")
        self.assertEqual(status.values[1].key, "docker0: MTU")
        self.assertEqual(status.values[1].value, "1500")
        self.assertEqual(status.values[2].key, "docker0: Receive OK")
        self.assertEqual(status.values[2].value, "0")
        self.assertEqual(status.values[3].key, "docker0: Receive ERR")
        self.assertEqual(status.values[3].value, "0")
        self.assertEqual(status.values[4].key, "docker0: Receive DRP")
        self.assertEqual(status.values[4].value, "0")
        self.assertEqual(status.values[5].key, "docker0: Receive OVR")
        self.assertEqual(status.values[5].value, "0")
        self.assertEqual(status.values[6].key, "docker0: Send OK")
        self.assertEqual(status.values[6].value, "0")
        self.assertEqual(status.values[7].key, "docker0: Send ERR")
        self.assertEqual(status.values[7].value, "0")
        self.assertEqual(status.values[8].key, "docker0: Send DRP")
        self.assertEqual(status.values[8].value, "0")
        self.assertEqual(status.values[9].key, "docker0: Send OVR")
        self.assertEqual(status.values[9].value, "0")
        self.assertEqual(status.values[10].key, "docker0: Flg")
        self.assertEqual(status.values[10].value, "BMU")

    def test_ok_full_items_abbreviated_target(self):
        """TESTCASE 26: When the interface name is omitted in netstat -i (HSRC)"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["enp0s31f6"],
                "items": [
                    "Interface",
                    "MTU",
                    "ReceiveOK",
                    "ReceiveERR",
                    "ReceiveDRP",
                    "ReceiveOVR",
                    "SendOK",
                    "SendERR",
                    "SendDRP",
                    "SendOVR",
                    "Flg",
                ],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface      MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0   1500         0      0      0 0             0    0    0    0 BMU
enp0s31f  1500    189108      0 175593 0           404    0    0    0 BMRU
ens15     1500    445362      0      0 0        223953    0    0    0 BMRU
lo       65536  68822265      0      0 0      68822265    0    0    0 LRU

"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "enp0s31f6: RXOK/189108 TXOK/404")
        self.assertEqual(len(status.values), 44)
        # Check only enp0s31f6
        self.assertEqual(status.values[11].key, "enp0s31f6: Interface")
        self.assertEqual(status.values[11].value, "enp0s31f6")
        self.assertEqual(status.values[12].key, "enp0s31f6: MTU")
        self.assertEqual(status.values[12].value, "1500")
        self.assertEqual(status.values[13].key, "enp0s31f6: Receive OK")
        self.assertEqual(status.values[13].value, "189108")
        self.assertEqual(status.values[14].key, "enp0s31f6: Receive ERR")
        self.assertEqual(status.values[14].value, "0")
        self.assertEqual(status.values[15].key, "enp0s31f6: Receive DRP")
        self.assertEqual(status.values[15].value, "175593")
        self.assertEqual(status.values[16].key, "enp0s31f6: Receive OVR")
        self.assertEqual(status.values[16].value, "0")
        self.assertEqual(status.values[17].key, "enp0s31f6: Send OK")
        self.assertEqual(status.values[17].value, "404")
        self.assertEqual(status.values[18].key, "enp0s31f6: Send ERR")
        self.assertEqual(status.values[18].value, "0")
        self.assertEqual(status.values[19].key, "enp0s31f6: Send DRP")
        self.assertEqual(status.values[19].value, "0")
        self.assertEqual(status.values[20].key, "enp0s31f6: Send OVR")
        self.assertEqual(status.values[20].value, "0")
        self.assertEqual(status.values[21].key, "enp0s31f6: Flg")
        self.assertEqual(status.values[21].value, "BMRU")

    def test_ok_part_items(self):
        """TESTCASE 27: Specify some items, normal case"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["ens15"],
                "items": ["ReceiveOK", "ReceiveERR", "SendOK", "SendERR"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface   MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0    1500         0      0      0 0             0    0    0    0 BMU
enp0s31f6  1500    189108      0 175593 0           404    0    0    0 BMRU
ens15      1500    445362      0      0 0        223953    0    0    0 BMRU
lo        65536  68822265      0      0 0      68822265    0    0    0 LRU
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "ens15: RXOK/445362 TXOK/223953")
        self.assertEqual(len(status.values), 16)

    def test_ok_multi_targets_all(self):
        """TESTCASE 28: Specify multiple targets and all match (display the result of the first matching target in the diagram)"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["enp0s31f6", "ens15", "lo"],
                "items": ["ReceiveOK", "SendOK", "SendERR"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface   MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0    1500         0      0      0 0             0    0    0    0 BMU
enp0s31f6  1500    189108      0 175593 0           404    0    0    0 BMRU
ens15      1500    445362      0      0 0        223953    0    0    0 BMRU
lo        65536  68822265      0      0 0      68822265    0    0    0 LRU
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "enp0s31f6: RXOK/189108 TXOK/404")
        self.assertEqual(len(status.values), 12)

    def test_ok_multi_targets_part(self):
        """TESTCASE 29: Specify multiple targets and some match (display the result of the first matching target in the diagram)"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["wlan0", "docker0", "ens15"],
                "items": ["ReceiveOK", "SendERR"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface   MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0    1500         0      0      0 0             0    0    0    0 BMU
enp0s31f6  1500    189108      0 175593 0           404    0    0    0 BMRU
ens15      1500    445362      0      0 0        223953    0    0    0 BMRU
lo        65536  68822265      0      0 0      68822265    0    0    0 LRU
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "docker0: RXOK/0 TXOK/0")
        self.assertEqual(len(status.values), 8)

    def test_ok_no_target_interface(self):
        """TESTCASE 30: OK even if the target interface does not exist"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["enp0s31f6", "wlan0"],
                "items": ["ReceiveOK", "ReceiveERR", "SendOK", "SendERR"],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface   MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0    1500         0      0      0 0             0    0    0    0 BMU
enp0s31f7  1500    189108      0 175593 0           404    0    0    0 BMRU
ens15      1500    445362      0      0 0        223953    0    0    0 BMRU
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "enp0s31f6, wlan0 not found")
        self.assertEqual(len(status.values), 12)

    def test_ok_no_statistics_available(self):
        """TESTCASE 31: When "no statistics available" is included"""
        m = tmc_computer_monitor.NetstatCommandMonitor(
            "Network",
            {
                "target": ["eth1"],
                "items": [
                    "Interface",
                    "MTU",
                    "ReceiveOK",
                    "ReceiveERR",
                    "ReceiveDRP",
                    "ReceiveOVR",
                    "SendOK",
                    "SendERR",
                    "SendDRP",
                    "SendOVR",
                    "Flg",
                ],
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""Kernel Interface table
Iface   MTU   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
docker0    1500       0      0      0 0             0     0     0      0 BMU
eth0       1500       1      0      0 0             8     0     0      0 BMRU
eth1       1500       0      0      0 0            52     0     0      0 BMRU
eth1:avahi  1500       - no statistics available -                      BMRU
lo        65536   10669      0      0 0         10669     0     0      0 LRU
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "eth1: RXOK/0 TXOK/52")
        self.assertEqual(len(status.values), 44)

    def test_ng_invalid_config_item(self):
        """TESTCASE 32: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.NetstatCommandMonitor(
                "Network", {"target": ["enp0s31f6"], "items": ["dummy"]}
            )
        self.assertEqual(str(ex.exception), "Unknown item: dummy in Network")

    def test_ng_target_is_not_list(self):
        """TESTCASE 33: Target is not a list"""
        with self.assertRaises(ValueError) as ex:
            tmc_computer_monitor.NetstatCommandMonitor(
                "Network", {"target": "wlan0", "items": ["ReceiveOK", "SendERR"]}
            )
        self.assertEqual(str(ex.exception), "target of Network is not list")

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


class IwconfigCommandMonitor(unittest.TestCase):
    def test_ok_full_items(self):
        """TESTCASE 37: Specify all items and normal case (insert actual command result)"""
        m = tmc_computer_monitor.IwconfigCommandMonitor(
            "Wireless",
            {
                "items": [
                    "IEEE",
                    "ESSID",
                    "Nickname",
                    "Mode",
                    "Frequency",
                    "AccessPointMac",
                    "BitRate",
                    "TxPower",
                    "Sensitivity",
                    "RetryMinLimit",
                    "RTSThr",
                    "FragmentThr",
                    "PowerManagement",
                    "LinkQuality",
                    "SignalLevel",
                    "NoiseLevel",
                    "RxInvalidNwid",
                    "InvalidCrypt",
                    "RxInvalidFrag",
                    "TxExcessiveRetries",
                    "InvalidMisc",
                    "MissedBeacon",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""eth0      no wireless extensions.

wlan0     IEEE 802.11abgn  ESSID:"HSR-11AC-09"
          Mode:Managed  Frequency:5.22 GHz  Access Point: E8:FC:AF:FF:61:33
          Bit Rate=87.8 Mb/s   Tx-Power=22 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=49/70  Signal level=-61 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:3  Invalid misc:660   Missed beacon:0

docker0   no wireless extensions.

eth1      no wireless extensions.

lo        no wireless extensions.
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(
            status.message, '"HSR-11AC-09" Bit Rate: 87.8 Mb/s  Signal: -61 dBm'
        )
        self.assertEqual(len(status.values), 17)
        # Check only the first key
        self.assertEqual(status.values[0].key, "IEEE")
        self.assertEqual(status.values[0].value, "IEEE 802.11abgn")
        self.assertEqual(status.values[1].key, "ESSID")
        self.assertEqual(status.values[1].value, '"HSR-11AC-09"')
        self.assertEqual(status.values[2].key, "Mode")
        self.assertEqual(status.values[2].value, "Managed")
        self.assertEqual(status.values[3].key, "Frequency")
        self.assertEqual(status.values[3].value, "5.22 GHz")
        self.assertEqual(status.values[4].key, "Access point MAC")
        self.assertEqual(status.values[4].value, " E8:FC:AF:FF:61:33")
        self.assertEqual(status.values[5].key, "Bit rate")
        self.assertEqual(status.values[5].value, "87.8 Mb/s")
        self.assertEqual(status.values[6].key, "Tx power")
        self.assertEqual(status.values[6].value, "22 dBm")
        self.assertEqual(status.values[7].key, "RTS thr")
        self.assertEqual(status.values[7].value, "off")
        self.assertEqual(status.values[8].key, "Fragment thr")
        self.assertEqual(status.values[8].value, "off")
        self.assertEqual(status.values[9].key, "Power management")
        self.assertEqual(status.values[9].value, "off")
        self.assertEqual(status.values[10].key, "Link quality")
        self.assertEqual(status.values[10].value, "49/70")
        self.assertEqual(status.values[11].key, "Signal level")
        self.assertEqual(status.values[11].value, "-61 dBm")
        self.assertEqual(status.values[12].key, "Rx invalid nwid")
        self.assertEqual(status.values[12].value, "0")
        self.assertEqual(status.values[13].key, "Rx invalid frag")
        self.assertEqual(status.values[13].value, "0")
        self.assertEqual(status.values[14].key, "Tx excessive retries")
        self.assertEqual(status.values[14].value, "3")
        self.assertEqual(status.values[15].key, "Invalid misc")
        self.assertEqual(status.values[15].value, "660")
        self.assertEqual(status.values[16].key, "Missed beacon")
        self.assertEqual(status.values[16].value, "0")

    def test_ok_part_items(self):
        """TESTCASE 38: Specify some items and normal case"""
        m = tmc_computer_monitor.IwconfigCommandMonitor(
            "Wireless",
            {
                "items": [
                    "IEEE",
                    "ESSID",
                    "BitRate",
                    "SignalLevel",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""wlp3s0    IEEE 802.11abgn  ESSID:"HSR-11AC-09"
          Mode:Managed  Frequency:5.22 GHz  Access Point: E8:FC:AF:FF:61:33
          Bit Rate=650 Mb/s   Tx-Power=22 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=59/70  Signal level=-51 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:5  Invalid misc:916   Missed beacon:0

lo        no wireless extensions.

enp4s0    no wireless extensions.

eno1      no wireless extensions.

docker0   no wireless extensions.
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(
            status.message, '"HSR-11AC-09" Bit Rate: 650 Mb/s  Signal: -51 dBm'
        )
        self.assertEqual(len(status.values), 4)

    def test_ok_eth_mode_and_disconnect(self):
        """TESTCASE 39: In case of eth mode or network disconnection"""
        m = tmc_computer_monitor.IwconfigCommandMonitor(
            "Wireless",
            {
                "items": [
                    "ESSID",
                    "BitRate",
                    "SignalLevel",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""eth0      no wireless extensions.

wlan0     IEEE 802.11abgn  ESSID:off/any
          Mode:Managed  Access Point: Not-Associated   Tx-Power=0 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off

docker0   no wireless extensions.

eth1      no wireless extensions.

lo        no wireless extensions.
"""
        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "Not connected")
        self.assertEqual(len(status.values), 1)

    def test_ng_invalid_config_item(self):
        """TESTCASE 40: Invalid item"""
        with self.assertRaises(RuntimeError) as ex:
            tmc_computer_monitor.IwconfigCommandMonitor(
                "Wireless", {"items": ["dummy"]}
            )
        self.assertEqual(str(ex.exception), "Unknown item: dummy in Wireless")

    def test_ng_insufficient_result(self):
        """TESTCASE 41: Command execution result cannot be parsed (result is insufficient)"""
        m = tmc_computer_monitor.IwconfigCommandMonitor(
            "Wireless",
            {
                "items": [
                    "IEEE",
                    "ESSID",
                    "BitRate",
                    "SignalLevel",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""eth0      no wireless extensions.

wlan0     IEEE 802.11abgn
          Mode:Managed  Frequency:5.22 GHz  Access Point: E8:FC:AF:FF:61:33
          Bit Rate=87.8 Mb/s   Tx-Power=22 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=49/70  Signal level=-61 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0

docker0   no wireless extensions.

eth1      no wireless extensions.

lo        no wireless extensions.
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "KeyError")
        self.assertEqual(status.values[1].value, "'ESSID'")

    def test_ng_multi_wireless_network(self):
        """TESTCASE 42: When there are multiple wireless networks"""
        m = tmc_computer_monitor.IwconfigCommandMonitor(
            "Wireless",
            {
                "items": [
                    "IEEE",
                    "ESSID",
                    "BitRate",
                    "SignalLevel",
                ]
            },
        )
        m._execute_command = mock.Mock()
        m._execute_command.return_value = r"""wlp3s0    IEEE 802.11abgn  ESSID:"HSR-11AC-09"
          Mode:Managed  Frequency:5.22 GHz  Access Point: E8:FC:AF:FF:61:33
          Bit Rate=650 Mb/s   Tx-Power=22 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=59/70  Signal level=-51 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:5  Invalid misc:916   Missed beacon:0

wlan0     IEEE 802.11abgn
          Mode:Managed  Frequency:5.22 GHz  Access Point: E8:FC:AF:FF:61:33
          Bit Rate=87.8 Mb/s   Tx-Power=22 dBm
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=49/70  Signal level=-61 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:5  Invalid misc:916   Missed beacon:0

docker0   no wireless extensions.

eth1      no wireless extensions.

lo        no wireless extensions.
"""

        status = m.get_diag().status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.values[0].value, "RuntimeError")
        self.assertEqual(
            status.values[1].value, "Multiple wireless networks are not supported"
        )

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

from diagnostic_msgs.msg import DiagnosticStatus

from .command_monitor import CommandMonitor


DIAG_PREFIX = "tmc_computer_monitor/"


class UdevAdmCommandMonitor(CommandMonitor):
    """Class to diagnose USB-connected devices"""

    def __init__(self, name: str, config: dict, prefix=DIAG_PREFIX):
        self._command = ["/bin/udevadm", "info"]
        super(UdevAdmCommandMonitor, self).__init__(name, config, prefix)

    def _check_config(self) -> None:
        super()._check_config()
        if "device" in self._config:
            self._command = ["/bin/udevadm", "info", self._config["device"]]

    def _parse(self, result) -> DiagnosticStatus:
        if isinstance(result, list):
            if "Unknown device" in result[0]:
                raise RuntimeError("Unknown device.")
            if "name or path is required" in result[0]:
                raise RuntimeError("A device name or path is required")
            return DiagnosticStatus(level=DiagnosticStatus.OK, message="OK")
        return DiagnosticStatus(level=DiagnosticStatus.ERROR, message="ERROR")

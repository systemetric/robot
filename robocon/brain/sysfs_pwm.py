import os


class PWMChannel:
    def __init__(
        self,
        chip: str,
        chan: int,
        enable: bool = True,
        freq: int = 50000,
        duty_cycle: float = 0,
    ):
        self._chip = chip
        self._chan = chan
        self._fd_enable = os.open(
            f"/sys/class/pwm/{chip}/pwm{chan}/enable", os.O_WRONLY
        )
        self._fd_period = os.open(
            f"/sys/class/pwm/{chip}/pwm{chan}/period", os.O_WRONLY
        )
        self._fd_duty_cycle = os.open(
            f"/sys/class/pwm/{chip}/pwm{chan}/duty_cycle", os.O_WRONLY
        )

        self._freq_hz = 0
        self._period = 0
        self._dc_percent = 0

        self.frequency = freq
        self.duty_cycle = duty_cycle
        self.enabled = enable

    @property
    def enabled(self) -> bool:
        return self._enable

    @enabled.setter
    def enabled(self, enable: bool):
        self._enable = enable
        if enable:
            os.write(self._fd_enable, b"1")
        else:
            os.write(self._fd_enable, b"0")

    @property
    def frequency(self) -> int:
        return self._freq_hz

    @frequency.setter
    def frequency(self, freq: int):
        self._freq_hz = int(freq)
        self._period = int(1e9 / self._freq_hz)  # sysfs uses period in nanoseconds
        os.write(self._fd_period, str(self._period).encode("utf-8"))

    @property
    def duty_cycle(self) -> float:
        return self._dc_percent

    @duty_cycle.setter
    def duty_cycle(self, duty_cycle: float):
        self._dc_percent = float(duty_cycle)
        dc = int(self._dc_percent * self._period / 100)
        os.write(self._fd_duty_cycle, str(dc).encode("utf-8"))

    def __del__(self):
        os.close(self._fd_duty_cycle)
        os.close(self._fd_period)
        os.close(self._fd_enable)


class PWMChip:
    def __init__(self, chip: str):
        self._chip = chip
        self._fd_export = os.open(f"/sys/class/pwm/{self._chip}/export", os.O_WRONLY)

    def _export_channel(self, chan: int):
        # trying to export if it already exists fails with EBUSY
        if os.access(f"/sys/class/pwm/{self._chip}/pwm{chan}", os.F_OK):
            return
        s_chan = str(chan).encode("utf-8")
        os.write(self._fd_export, s_chan)

    def channel(self, chan: int, **kwargs) -> PWMChannel:
        self._export_channel(chan)
        return PWMChannel(self._chip, chan, **kwargs)

    def __del__(self):
        os.close(self._fd_export)

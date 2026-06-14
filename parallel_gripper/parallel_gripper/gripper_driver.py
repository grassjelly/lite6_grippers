class ParallelGripperOpenRB150:
    """Parallel gripper driver using xArm tool GPIO Modbus RTU.

    Hardware: M5Stack Atom Lite + Feetech SCS15 servo.
    Communication: Modbus RTU at 115200 baud over xArm tool serial port.

    Position convention: 0.0 m = fully closed, 0.025 m = fully open.
    Each finger moves symmetrically; the driver node reports drive_joint position
    which equals the displacement of each individual finger.
    """

    CLOSE_POS = 0    # servo raw position (closed)
    OPEN_POS = 512   # servo raw position (open)
    MAX_METERS = 0.025

    def __init__(self, arm):
        self._arm = arm
        self._arm.set_mode(0)
        self._arm.set_state(0)
        code = self._arm.set_tgpio_modbus_baudrate(115200)
        if code != 0:
            raise RuntimeError(f'set_tgpio_modbus_baudrate failed, code={code}')
        self._arm.set_tgpio_modbus_timeout(50)
        self._arm.set_tgpio_digital(0, 1)
        self._arm.set_tgpio_digital(1, 1)

    def _meters_to_raw(self, pos_m):
        pos_m = max(0.0, min(self.MAX_METERS, pos_m))
        return int(pos_m / self.MAX_METERS * (self.OPEN_POS - self.CLOSE_POS) + self.CLOSE_POS)

    def _raw_to_meters(self, raw):
        return (raw - self.CLOSE_POS) / (self.OPEN_POS - self.CLOSE_POS) * self.MAX_METERS

    def _move_raw(self, raw):
        raw = max(self.CLOSE_POS, min(self.OPEN_POS, raw))
        data = [0x01, 0x06, 0x00, 0x80, raw >> 8, raw & 0xFF]
        self._arm.getset_tgpio_modbus_data(data)

    def move(self, pos_m):
        self._move_raw(self._meters_to_raw(pos_m))

    def open(self):
        self._move_raw(self.OPEN_POS)

    def close(self):
        self._move_raw(self.CLOSE_POS)

    def get_pos(self):
        cmd = [0x01, 0x03, 0x01, 0x01, 0x00, 0x01]
        code, resp = self._arm.getset_tgpio_modbus_data(cmd)
        if code != 0:
            raise RuntimeError(f'xArm SDK error code={code}')
        raw = (resp[3] << 8) | resp[4]
        return self._raw_to_meters(raw)

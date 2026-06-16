from xarm_msgs.srv import GetSetModbusData


class ParallelGripperOpenRB150:
    """Parallel gripper driver via xArm tool GPIO Modbus RTU ROS2 service.

    Hardware: M5Stack Atom Lite + Feetech SCS15 servo.
    Communication: Modbus RTU at 115200 baud over xArm tool serial port,
    routed through the ufactory_driver getset_tgpio_modbus_data service.

    Position convention: 0.0 m = fully closed, 0.025 m = fully open.
    Each finger moves symmetrically; the driver node reports drive_joint position
    which equals the displacement of each individual finger.
    """

    CLOSE_POS = 512    # servo raw position (closed)
    OPEN_POS = 0   # servo raw position (open)
    MAX_METERS = 0.025

    def __init__(self, modbus_client):
        self._client = modbus_client

    def _meters_to_raw(self, pos_m):
        pos_m = max(0.0, min(self.MAX_METERS, pos_m))
        return int(pos_m / self.MAX_METERS * (self.OPEN_POS - self.CLOSE_POS) + self.CLOSE_POS)

    def _raw_to_meters(self, raw):
        return (raw - self.CLOSE_POS) / (self.OPEN_POS - self.CLOSE_POS) * self.MAX_METERS

    def move_async(self, pos_m):
        """Return a Future for a Modbus write-register command to move to pos_m."""
        raw = self._meters_to_raw(pos_m)  # already clamped inside _meters_to_raw
        req = GetSetModbusData.Request()
        req.modbus_data = [0x01, 0x06, 0x00, 0x80, raw >> 8, raw & 0xFF]
        req.ret_length = 8  # echo of write command (6 bytes + 2 CRC)
        return self._client.call_async(req)

    def get_pos_async(self):
        """Return a Future for a Modbus read-register command to get current position."""
        req = GetSetModbusData.Request()
        req.modbus_data = [0x01, 0x03, 0x01, 0x01, 0x00, 0x01]
        req.ret_length = 7  # addr + func + count + 2 data bytes + 2 CRC
        return self._client.call_async(req)

    def parse_pos(self, response):
        """Parse a get_pos_async response into meters."""
        if response.ret != 0:
            raise RuntimeError(f'getset_tgpio_modbus_data error ret={response.ret}')
        if len(response.ret_data) < 5:
            raise RuntimeError(f'Short Modbus response: {list(response.ret_data)}')
        raw = (response.ret_data[3] << 8) | response.ret_data[4]
        return self._raw_to_meters(raw)

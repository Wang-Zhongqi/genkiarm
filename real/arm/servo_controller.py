"""Basic serial control used by the servo slider script."""
import serial


class ServoController:
    def __init__(self, port, baudrate=1000000, timeout=.1):
        self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def close(self):
        self.serial_port.close()

    def _send_packet(self, servo_id, parameters):
        core = [servo_id, len(parameters) + 2, 3, *parameters]
        packet = bytes([255, 255, *core, (~sum(core)) & 255])
        try:
            written = self.serial_port.write(packet)
            self.serial_port.flush()
            return written == len(packet)
        except serial.SerialException:
            return False

    def enable_torque(self, servo_id):
        return self._send_packet(servo_id, [40, 1])

    def set_servo_angle(self, servo_id, angle):
        if not -90 <= angle <= 90:
            return False
        position = int(round(2048 + angle * 4096 / 360))
        return self._send_packet(servo_id, [42, position & 255, position >> 8])

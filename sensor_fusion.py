import serial
import re

def connect_serial(port, baud=115200):
    return serial.Serial(port, baud, timeout=1)


class SensorFallDetector:
    def __init__(self, port, baud=115200):
        self.ser = connect_serial(port, baud)
        self.fall_detected = False
        self.confidence = None
        self.impact_g = None
        self.gyro_peak_dps = None
        self.orientation_change_deg = None

    def check_for_fall(self):   #check if arduino send sth new
        if self.ser.in_waiting == 0: #if no data we do return bye bye
            return  # nothing new to read right now

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            return

        if not line:  #if line comes back w nothing useful byebye
            return

        if line == "FALL_DETECTED":   #yay sth useful oops
            self.fall_detected = True
            return

        if line.startswith("Impact="):
            match = re.search(
                r"Impact=([\d.]+)g \| GyroPeak=([\d.]+) dps \| OrientationChange=([\d.]+) deg \| Confidence=(\d+)%",
                line
            )
            if match:
                self.impact_g = float(match.group(1))
                self.gyro_peak_dps = float(match.group(2))
                self.orientation_change_deg = float(match.group(3))
                self.confidence = int(match.group(4))
            return 
        print(f"[unhandled line]: {line}")

    def reset(self):   
        """Call this after handling a confirmed fall, to listen for the next one."""
        self.fall_detected = False
        self.confidence = 0


if __name__ == "__main__":
    PORT = "COM9"  #placeholder anvita pls replace
    sensor = SensorFallDetector(PORT)

    print("Listening to Arduino... (Ctrl+C to stop)")

    try:
        while True:
            sensor.check_for_fall()
            if sensor.fall_detected:
                print(f"FALL DETECTED! Confidence: {sensor.confidence}%")
                sensor.reset()
    except KeyboardInterrupt:
        print("Stopped.")
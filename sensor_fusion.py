import serial


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

    def check_for_fall(self):

        if self.ser.in_waiting == 0:
            return

        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            print("Received:", line)       # Debug
        except Exception as e:
            print(e)
            return

        if line != "FALL_DETECTED":
            return

        try:
            impact = self.ser.readline().decode().strip()
            gyro = self.ser.readline().decode().strip()
            orientation = self.ser.readline().decode().strip()
            confidence = self.ser.readline().decode().strip()

            print(impact)
            print(gyro)
            print(orientation)
            print(confidence)

            self.impact_g = float(impact.split("=")[1])
            self.gyro_peak_dps = float(gyro.split("=")[1])
            self.orientation_change_deg = float(orientation.split("=")[1])
            self.confidence = int(confidence.split("=")[1])

            self.fall_detected = True

        except Exception as e:
            print("Parsing Error:", e)

    def reset(self):
        self.fall_detected = False
        self.confidence = None
        self.impact_g = None
        self.gyro_peak_dps = None
        self.orientation_change_deg = None


if __name__ == "__main__":

    PORT = "COM9"      # Change if needed

    sensor = SensorFallDetector(PORT)

    print("Listening to Arduino...")

    try:
        while True:
            sensor.check_for_fall()

            if sensor.fall_detected:
                print("\n===== FALL DETECTED =====")
                print(f"Impact: {sensor.impact_g} g")
                print(f"Gyro Peak: {sensor.gyro_peak_dps} dps")
                print(f"Orientation Change: {sensor.orientation_change_deg}°")
                print(f"Confidence: {sensor.confidence}%")
                print("=========================\n")

                sensor.reset()

    except KeyboardInterrupt:
        sensor.ser.close()
        print("Stopped.")
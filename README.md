Fall Detection System

This project is a fall detection prototype built using an Arduino Uno, an MPU6050 accelerometer + gyroscope sensor, and Python/C++-based sensor processing. The goal is to detect sudden motion patterns that are consistent with a fall and then report the event through serial output for further analysis or integration.

Overview

The system continuously reads motion data from the MPU6050 and uses a simple state-machine approach to detect a fall based on:

- sudden low acceleration / possible free fall
- impact spike
- significant rotation
- short post-impact stillness

The sensor data is printed in a structured serial format so it can be used by other programs such as Python scripts or monitoring tools.

Features

- Reads real-time accelerometer and gyroscope data
- Calibrates gyro offset at startup
- Detects possible fall events using a multi-stage logic
- Prints live motion readings to Serial Monitor
- Freezes output for 10 seconds after a fall is detected
- Suitable for later integration with OpenCV / MediaPipe based vision detection

Hardware Used

- Arduino Uno
- MPU6050 sensor module
- USB connection to laptop/PC

Serial Output Format

Normal readings are printed like this:

```text
ACCEL: X=0.90g Y=0.33g Z=0.18g | GYRO: X=-1.66 Y=-0.06 Z=-1.23
````

When a fall is detected, the output is printed in a structured format such as:

```text
FALL_DETECTED
Impact=...
GyroPeak=...
OrientationChange=...
Confidence=...
```

How It Works

1. The MPU6050 is initialized over I2C.
2. Gyroscope calibration is performed while the sensor is still.
3. The program reads acceleration and rotation values continuously.
4. A fall is detected when the motion pattern matches the defined thresholds.
5. The system prints the fall event and pauses further output for a short time.

Notes

* Thresholds may need tuning depending on sensor placement.
* The current setup works best for debugging on a table first.
* Later, the sensor can be mounted on an arm cuff or wearable band for real-world testing.

Future Improvements

* Better threshold tuning using real sensor data
* Python serial reader for event logging
* Vision-based confirmation using OpenCV/MediaPipe
* Mobile or web dashboard for alerts

```

If you want, I can also :contentReference[oaicite:0]{index=0}.
```

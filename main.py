import cv2
from vision_fall import VisionFallDetector
from sensor_fusion import SensorFallDetector
from telegram_alert import send_telegram_alert

VISION_DETECTOR = VisionFallDetector()

SENSOR_PORT = "COM9"
sensor = SensorFallDetector(SENSOR_PORT)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Running fall detection... press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    frame, vision_fall_like, body_angle = VISION_DETECTOR.process_frame(frame)

    sensor.check_for_fall()

    if sensor.fall_detected:
        print(f"CONFIRMED FALL — Sensor Confidence: {sensor.confidence}%, "
              f"Impact: {sensor.impact_g}g, "
              f"Vision Fall-like: {vision_fall_like}, Body Angle: {body_angle}")


        message = (
            f"🚨 Fall Detected!\n"
            f"Sensor Confidence: {sensor.confidence}%\n"
            f"Impact: {sensor.impact_g}g\n"
            f"Vision confirms lying posture: {vision_fall_like}"
        )
    send_telegram_alert(message)

    sensor.reset()


    cv2.imshow("Fall Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
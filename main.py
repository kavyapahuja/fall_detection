import cv2
from vision_fall import VisionFallDetector

detector = VisionFallDetector()

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

    frame, fall_like, body_angle = detector.process_frame(frame)

    cv2.imshow("Fall Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
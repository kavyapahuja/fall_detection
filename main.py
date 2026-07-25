import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = python.BaseOptions
PoseLandmarker = vision.PoseLandmarker
PoseLandmarkerOptions = vision.PoseLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='pose_landmarker.task'),
    running_mode=VisionRunningMode.VIDEO
)

landmarker = PoseLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

frame_timestamp = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    result = landmarker.detect_for_video(mp_image, frame_timestamp)
    frame_timestamp += 1

    if result.pose_landmarks:
        h, w, _ = frame.shape
        landmarks = result.pose_landmarks[0]  # first detected person

        # MediaPipe landmark indices we care about
        LEFT_SHOULDER = 11
        RIGHT_SHOULDER = 12
        LEFT_HIP = 23
        RIGHT_HIP = 24
        NOSE = 0
        LEFT_ANKLE = 27
        RIGHT_ANKLE = 28
        LEFT_KNEE = 27
        RIGHT_KNEE = 28

        key_points = {
            "nose": landmarks[NOSE],
            "left_shoulder": landmarks[LEFT_SHOULDER],
            "right_shoulder": landmarks[RIGHT_SHOULDER],
            "left_hip": landmarks[LEFT_HIP],
            "right_hip": landmarks[RIGHT_HIP],
            "left_ankle": landmarks[LEFT_ANKLE],
            "right_ankle": landmarks[RIGHT_ANKLE],
            "right_knee" : landmarks[RIGHT_KNEE],
            "left_knee" : landmarks[LEFT_KNEE],
        }

        for name, lm in key_points.items():
            x, y = int(lm.x * w), int(lm.y * h)
            cv2.circle(frame, (x, y), 6, (0, 255, 0), -1)
            cv2.putText(frame, name, (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        # print raw coordinates for now, so we can see the numbers
        print(key_points["nose"])

    cv2.imshow("Fall Detection - Pose Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
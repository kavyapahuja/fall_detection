import cv2
import mediapipe as mp
import math
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

def calculate_body_angle(shoulder_mid, hip_mid):
    dx = hip_mid[0] - shoulder_mid[0]
    dy = hip_mid[1] - shoulder_mid[1]
    angle = math.degrees(math.atan2(abs(dy), abs(dx)))
    return angle

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
    print("Error: Could not open video")
    exit()

frame_timestamp = 0

ANGLE_THRESHOLD = 50       # below this = horizontal/fallen posture
SUSTAIN_FRAMES = 15        # must stay horizontal for this many frames in a row (~1 sec)
low_angle_counter = 0
fall_like = False

no_detection_counter = 0
MAX_NO_DETECTION = 30      # allow ~1 sec of lost tracking before un-flagging

# landmark indices
NOSE = 0
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_HIP, RIGHT_HIP = 23, 24
LEFT_KNEE, RIGHT_KNEE = 25, 26
LEFT_ANKLE, RIGHT_ANKLE = 27, 28

while True:
    ret, frame = cap.read()
    if not ret:
        print("Video ended")
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    result = landmarker.detect_for_video(mp_image, frame_timestamp)
    frame_timestamp += 1

    if result.pose_landmarks:
        no_detection_counter = 0  # reset, since we detected a person again

        h, w, _ = frame.shape
        landmarks = result.pose_landmarks[0]

        key_points = {
            "nose": landmarks[NOSE],
            "left_shoulder": landmarks[LEFT_SHOULDER],
            "right_shoulder": landmarks[RIGHT_SHOULDER],
            "left_hip": landmarks[LEFT_HIP],
            "right_hip": landmarks[RIGHT_HIP],
            "left_knee": landmarks[LEFT_KNEE],
            "right_knee": landmarks[RIGHT_KNEE],
            "left_ankle": landmarks[LEFT_ANKLE],
            "right_ankle": landmarks[RIGHT_ANKLE],
        }

        # draw all landmark dots + labels
        for name, lm in key_points.items():
            x, y = int(lm.x * w), int(lm.y * h)
            cv2.circle(frame, (x, y), 9, (0, 255, 0), -1)
            cv2.putText(frame, name, (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # midpoints for angle calculation
        shoulder_mid = (
            (key_points["left_shoulder"].x + key_points["right_shoulder"].x) / 2,
            (key_points["left_shoulder"].y + key_points["right_shoulder"].y) / 2
        )
        hip_mid = (
            (key_points["left_hip"].x + key_points["right_hip"].x) / 2,
            (key_points["left_hip"].y + key_points["right_hip"].y) / 2
        )

        body_angle = calculate_body_angle(shoulder_mid, hip_mid)

        print(f"Shoulder visibility: L={key_points['left_shoulder'].visibility:.2f}, R={key_points['right_shoulder'].visibility:.2f}")
        print(f"Hip visibility: L={key_points['left_hip'].visibility:.2f}, R={key_points['right_hip'].visibility:.2f}")


        if body_angle < ANGLE_THRESHOLD:
            low_angle_counter += 1
        else:
            low_angle_counter = 0

        if low_angle_counter >= SUSTAIN_FRAMES:
            fall_like = True
        elif low_angle_counter == 0:
            fall_like = False

        cv2.putText(frame, f"Body Angle: {int(body_angle)}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    else:
        no_detection_counter += 1
        if no_detection_counter > MAX_NO_DETECTION:
            fall_like = False  # only un-flag after a real gap, not a flicker

    cv2.putText(frame, f"Fall-like: {fall_like}", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    print(f"low_angle_counter={low_angle_counter}, no_detection_counter={no_detection_counter}, fall_like={fall_like}")   # ← ADD THIS LINE HERE

    cv2.imshow("Fall Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
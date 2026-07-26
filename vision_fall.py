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

def setup_pose_landmarker(model_path='pose_landmarker.task'):
    BaseOptions = python.BaseOptions
    PoseLandmarker = vision.PoseLandmarker
    PoseLandmarkerOptions = vision.PoseLandmarkerOptions
    VisionRunningMode = vision.RunningMode

    options = PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.VIDEO
    )
    return PoseLandmarker.create_from_options(options)


class VisionFallDetector:
    def __init__(self, model_path='pose_landmarker.task',
                 angle_threshold=50, sustain_frames=15, max_no_detection=30):
        self.landmarker = setup_pose_landmarker(model_path)
        self.frame_timestamp = 0

        self.ANGLE_THRESHOLD = angle_threshold
        self.SUSTAIN_FRAMES = sustain_frames
        self.MAX_NO_DETECTION = max_no_detection

        self.low_angle_counter = 0
        self.no_detection_counter = 0
        self.fall_like = False
        self.body_angle = None

        self.NOSE = 0
        self.LEFT_SHOULDER, self.RIGHT_SHOULDER = 11, 12
        self.LEFT_HIP, self.RIGHT_HIP = 23, 24
        self.LEFT_KNEE, self.RIGHT_KNEE = 25, 26
        self.LEFT_ANKLE, self.RIGHT_ANKLE = 27, 28

    def process_frame(self, frame):
        """Takes one frame, returns (annotated_frame, fall_like, body_angle)."""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        result = self.landmarker.detect_for_video(mp_image, self.frame_timestamp)
        self.frame_timestamp += 1

        if result.pose_landmarks:
            self.no_detection_counter = 0

            h, w, _ = frame.shape
            landmarks = result.pose_landmarks[0]

            key_points = {
                "nose": landmarks[self.NOSE],
                "left_shoulder": landmarks[self.LEFT_SHOULDER],
                "right_shoulder": landmarks[self.RIGHT_SHOULDER],
                "left_hip": landmarks[self.LEFT_HIP],
                "right_hip": landmarks[self.RIGHT_HIP],
                "left_knee": landmarks[self.LEFT_KNEE],
                "right_knee": landmarks[self.RIGHT_KNEE],
                "left_ankle": landmarks[self.LEFT_ANKLE],
                "right_ankle": landmarks[self.RIGHT_ANKLE],
            }

            for name, lm in key_points.items():
                x, y = int(lm.x * w), int(lm.y * h)
                cv2.circle(frame, (x, y), 9, (0, 255, 0), -1)
                cv2.putText(frame, name, (x + 8, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            shoulder_mid = (
                (key_points["left_shoulder"].x + key_points["right_shoulder"].x) / 2,
                (key_points["left_shoulder"].y + key_points["right_shoulder"].y) / 2
            )
            hip_mid = (
                (key_points["left_hip"].x + key_points["right_hip"].x) / 2,
                (key_points["left_hip"].y + key_points["right_hip"].y) / 2
            )

            self.body_angle = calculate_body_angle(shoulder_mid, hip_mid)

            if self.body_angle < self.ANGLE_THRESHOLD:
                self.low_angle_counter += 1
            else:
                self.low_angle_counter = 0

            if self.low_angle_counter >= self.SUSTAIN_FRAMES:
                self.fall_like = True
            elif self.low_angle_counter == 0:
                self.fall_like = False

            cv2.putText(frame, f"Body Angle: {int(self.body_angle)}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        else:
            self.no_detection_counter += 1
            if self.no_detection_counter > self.MAX_NO_DETECTION:
                self.fall_like = False

        cv2.putText(frame, f"Fall-like: {self.fall_like}", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return frame, self.fall_like, self.body_angle
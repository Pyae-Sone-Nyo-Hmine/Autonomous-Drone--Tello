import mediapipe as mp
import cv2

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_holistic = mp.solutions.holistic
pose = mp_pose.Pose(min_tracking_confidence=0.65, min_detection_confidence=0.65)


def length(x1, y1, x2, y2):
    return abs(((y2 - y1) ** 2 + (x2 - x1) ** 2) ** 0.5)


class Pose:

    def __init__(self, img):
        self.img = img
        self.x, self.y, self.channels = img.shape
        self.imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = pose.process(self.imgRGB)
        self.landmarks = self.results.pose_landmarks
        if self.landmarks:
            self.nose = self.landmarks.landmark[mp_holistic.PoseLandmark.NOSE].x * self.x, \
                        self.landmarks.landmark[mp_holistic.PoseLandmark.NOSE].y * self.y
            self.right_shoulder = self.landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER].x * self.x, \
                                  self.landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER].y * self.y
            self.left_shoulder = self.landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER].x * self.x, \
                                 self.landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER].x * self.x
            self.right_hip = self.landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].x * self.x, \
                             self.landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].y * self.y
            self.left_hip = self.landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].x * self.x, \
                            self.landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].y * self.y


    def torso_area(self):
        if self.landmarks:
            shoulder_length = length(self.right_shoulder[0], self.right_shoulder[1], self.left_shoulder[0],
                                     self.left_shoulder[0])
            hip_length = length(self.right_hip[0], self.right_hip[1], self.left_hip[0], self.left_hip[1])
            h = abs((self.right_shoulder[1] + self.left_shoulder[1]) / 2 - (self.right_hip[1] + self.left_hip[1]) / 2)
            area = (1 / 2) * (shoulder_length + hip_length) * h

            self.torso_area = area
        else:
            self.torso_area = 0

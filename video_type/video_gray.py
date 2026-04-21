# video_gray.py
import cv2

class GrayProcessor:
    def __init__(self):
        pass

    def process(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # BGR
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
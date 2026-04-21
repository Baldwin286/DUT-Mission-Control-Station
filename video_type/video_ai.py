# video_ai.py
from ultralytics import YOLO

class AIProcessor:
    def __init__(self, model_path):
        print(f"Đang nạp model YOLO từ: {model_path}")
        self.model = YOLO(model_path)

    def process(self, frame):
        results = self.model.predict(frame, conf=0.4, imgsz=320, verbose=False, stream=False)
        return results[0].plot()
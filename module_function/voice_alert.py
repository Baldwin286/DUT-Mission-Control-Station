import pyttsx3
import queue
from PyQt6.QtCore import QThread

class TTSWorker(QThread):
    def __init__(self):
        super().__init__()
        self.q = queue.Queue()
        self.running = True

    def speak(self, text):
        self.q.put(text)

    def run(self):
        engine = pyttsx3.init()
        engine.setProperty('rate', 160)
        
        voices = engine.getProperty('voices')
        if len(voices) > 1:
            engine.setProperty('voice', voices[1].id) 
            
        while self.running:
            try:
                text = self.q.get(timeout=0.5) 
                engine.say(text)
                engine.runAndWait()
            except queue.Empty:
                pass

    def stop(self):
        self.running = False
        self.wait()
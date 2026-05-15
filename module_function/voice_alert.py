import queue
from PyQt6.QtCore import QThread

import pythoncom
import win32com.client

class TTSWorker(QThread):
    def __init__(self):
        super().__init__()
        self.q = queue.Queue()
        self.running = True

    def speak(self, text):
        if self.running:
            self.q.put(text)

    # def run(self):
    #     pythoncom.CoInitialize()
    #     try:
    #         speaker = win32com.client.Dispatch("SAPI.SpVoice")
            
    #         speaker.Rate = 1 
    #         speaker.Volume = 100
            
    #         while self.running:
    #             try:
    #                 text = self.q.get(timeout=0.1)

    #                 speaker.Speak(text)
    #             except queue.Empty:
    #                 pass
    #             except Exception as e:
    #                 print(f"[TTS Error] Voice reading error: {e}")
    #     finally:
    #         pythoncom.CoUninitialize()

    def run(self):
        pythoncom.CoInitialize()
        try:
            speaker = win32com.client.Dispatch("SAPI.SpVoice")
            
            voices = speaker.GetVoices()
            print("\n[TTS INFO] --- LIST OF AVAILABLE VOICES ---")
            for i in range(voices.Count):
                voice = voices.Item(i)
                voice_name = voice.GetDescription()  
                print(f"Voice {i}: {voice_name}")
                
                if "Zira" in voice_name or "Hazel" in voice_name:
                    speaker.Voice = voice
            print("------------------------------------------\n")
            
            speaker.Rate = 0.8  
            speaker.Volume = 100
            
            while self.running:
                try:
                    text = self.q.get(timeout=0.1)
                    speaker.Speak(text)
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"[TTS Error] Error while reading: {e}")
        except Exception as e:
            print(f"[TTS Critical] Voice initialization error: {e}")
        finally:
            pythoncom.CoUninitialize()

    def stop(self):
        self.running = False
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except queue.Empty:
                break
        self.wait()
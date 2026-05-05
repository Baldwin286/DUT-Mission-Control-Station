import cv2
import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay|er;1"

def main():
    video_url = "udp://@0.0.0.0:9999"
    print(f"[LAPTOP] Waiting for video connection from Pi at: {video_url}")

    cap = cv2.VideoCapture(video_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Error: Cannot open network stream. Check your Firewall!")
        return

    print("[LAPTOP] Connection successful! Decoding images...")

    while True:
        ret, frame = cap.read()
        if not ret:
            cv2.waitKey(10)
            continue

        cv2.imshow("TEST STREAM - Press 'q' to exit", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
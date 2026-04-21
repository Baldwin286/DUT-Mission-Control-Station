import struct 

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(object)

    def __init__(self, port=9999):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.bind(("0.0.0.0", self.port))
        server_sock.listen(1)
        server_sock.settimeout(1.0)
        
        print(f"[TCP] AI Stream Server đang mở cửa lắng nghe tại port {self.port}...")

        while self.running:
            try:
                # Waiting for Pi to connect
                conn, addr = server_sock.accept()
                print(f"\n[TCP] Drone đã kết nối thành công từ IP: {addr}")
                conn.settimeout(2.0)
                
                data = b""
                payload_size = struct.calcsize(">L") # Header size (4 bytes)

                while self.running:
                    try:
                        # 1. Get the 4-byte header to know the size of the upcoming image
                        while len(data) < payload_size:
                            packet = conn.recv(4096)
                            if not packet: break
                            data += packet
                        
                        if len(data) < payload_size:
                            break 
                            
                        packed_msg_size = data[:payload_size]
                        data = data[payload_size:]
                        msg_size = struct.unpack(">L", packed_msg_size)[0] # Decode to find the actual capacity

                        # 2. Continuously receive data until the image size is full
                        while len(data) < msg_size:
                            data += conn.recv(4096)
                            
                        frame_data = data[:msg_size]
                        data = data[msg_size:]

                        # 3. Decode and display on the interface
                        frame_np = np.frombuffer(frame_data, dtype=np.uint8)
                        frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

                        if frame is not None:
                            self.change_pixmap_signal.emit(frame)
                            
                    except socket.timeout:
                        # After the (normal) timeout period, return to the self.running loop to check
                        continue
                    except Exception as e:
                        print(f"\n[TCP] Drone mất kết nối: {e}")
                        break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Lỗi Server: {e}")

    def stop(self):
        self.running = False
        self.wait()
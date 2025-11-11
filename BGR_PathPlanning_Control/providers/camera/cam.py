import asyncio
import signal
import cv2
import base64
import logging

log = logging.getLogger("CameraProvider")

class CameraProvider:
    def __init__(self, port=8765):
        self.cap = cv2.VideoCapture(0)
        self.server = None
        self.port = port
        self.running = True

    async def start(self):
        pass

    async def stop(self):
        self.running = False
        self.cap.release()
        log.info("✅ Resources released. Goodbye!")

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            log.error("❌ Could not read frame from camera")
            return {}
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer.tobytes()).decode('utf-8')
        return {"frame": jpg_as_text}
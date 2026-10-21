"""USB camera source with the same interface as device/cameras.py (start, frame, errors, close).

One capture thread reads continuously so the driver buffer never goes stale, and encodes a
downscaled JPEG at encode_hz. Use device "test" for a synthetic pattern (bench tests).
"""

import hashlib
import threading
import time
from uuid import uuid4

from drivingbench.device.cameras import Frame, adjust, camera_clock


class USBCameras:
    def __init__(
        self,
        device="/dev/video0",
        width=1920,
        height=1080,
        fps=30,
        fourcc="MJPG",
        out_width=1280,
        quality=85,
        encode_hz=5.0,
        name="narrow",
    ):
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self.fourcc, self.out_width, self.quality = fourcc, out_width, quality
        self.encode_period = 1.0 / encode_hz
        self.name = name
        self.frames = {}
        self.errors = {}
        self.cache = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None

    # ---- capture -----------------------------------------------------------------------------
    def _open(self, cv2):
        if self.device == "test":
            return None
        source = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            raise OSError(f"cannot open camera {self.device}")
        return cap

    def _test_frame(self, np, cv2):
        image = np.zeros((self.height, self.width, 3), np.uint8)
        image[:] = (60, 90, 60)
        image[: self.height // 2] = (200, 160, 120)
        cv2.putText(image, time.strftime("%H:%M:%S"), (40, 120), cv2.FONT_HERSHEY_SIMPLEX,
                    3, (255, 255, 255), 6)
        time.sleep(1.0 / self.fps)
        return image

    def _encode(self, cv2, image, captured_at, generation, count):
        h, w = image.shape[:2]
        if self.out_width and w > self.out_width:
            image = cv2.resize(image, (self.out_width, round(h * self.out_width / w)),
                               interpolation=cv2.INTER_AREA)
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise ValueError("jpeg_encode_failed")
        frame = Frame(f"{generation}-{count}", captured_at, buffer.tobytes())
        with self.lock:
            self.frames[self.name] = frame
            self.errors.pop(self.name, None)

    def _run(self):
        import cv2
        import numpy as np

        backoff = 0.5
        while not self.stop.is_set():
            generation, count, last_encoded = uuid4().hex[:8], 0, 0.0
            cap = None
            try:
                cap = self._open(cv2)
                backoff = 0.5
                while not self.stop.is_set():
                    if cap is None:
                        image, ok = self._test_frame(np, cv2), True
                    else:
                        ok, image = cap.read()
                    # Read time stands in for exposure time; true capture is ~1 frame earlier.
                    now = camera_clock()
                    if not ok or image is None:
                        raise OSError("camera_read_failed")
                    if now - last_encoded >= self.encode_period:
                        count += 1
                        self._encode(cv2, image, now, generation, count)
                        last_encoded = now
            except Exception as error:
                with self.lock:
                    self.errors[self.name] = str(error)
                self.stop.wait(backoff)
                backoff = min(backoff * 2, 10.0)
            finally:
                if cap is not None:
                    cap.release()

    # ---- producer interface ------------------------------------------------------------------
    def start(self):
        self.thread = threading.Thread(target=self._run, name="usb-camera", daemon=True)
        self.thread.start()

    def put(self, camera, frame):
        with self.lock:
            self.frames[camera] = frame

    def frame(self, camera, adjustments):
        with self.lock:
            frame = self.frames.get(camera)
            if frame is None:
                return None
            key = (frame.key, adjustments.model_dump_json())
            cached = self.cache.get(camera)
            if cached and cached[0] == key:
                return cached[1]
        jpeg = adjust(frame.jpeg, adjustments)
        rendered = Frame(frame.key + "-" + hashlib.sha256(jpeg).hexdigest()[:12],
                         frame.captured_at, jpeg)
        with self.lock:
            self.cache[camera] = (key, rendered)
        return rendered

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)

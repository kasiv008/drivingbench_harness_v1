"""The cart's USB cameras (front, left, right, rear) and a last-good-frame cache.

One capture thread per camera reads continuously so the driver buffer never goes stale. grab()
only dequeues the MJPEG buffer; a frame is decoded, downscaled and re-encoded at encode_hz, so
four 1080p streams cost little CPU. Device "test" draws a synthetic, labelled pattern.

USB bandwidth: these cameras reserve the full isochronous bandwidth for MJPEG at any resolution,
so one USB 2.0 controller carries only two of them. Cameras start in CAMERA_NAMES order with a
short stagger, so front claims bandwidth first; a camera that cannot stream reports
"no_frames" in errors and keeps retrying with backoff. No motion dependencies.
"""

import hashlib
import io
import threading
import time
from dataclasses import dataclass
from uuid import uuid4

from drivingbench.shared.contracts import CAMERA_NAMES, ImageSettings
from PIL import Image, ImageEnhance

START_STAGGER_S = 0.3
NO_FRAMES_HINT = "no_frames (check USB bandwidth: two cameras per USB 2.0 controller)"


def camera_clock():
    return time.clock_gettime(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))


@dataclass(frozen=True)
class Frame:
    key: str
    captured_at: float
    jpeg: bytes


def adjust(jpeg, settings):
    if settings == ImageSettings():
        return jpeg
    with Image.open(io.BytesIO(jpeg)) as original:
        image = original.convert("RGB")
        image = ImageEnhance.Brightness(image).enhance(2**settings.exposure_ev)
        image = ImageEnhance.Contrast(image).enhance(settings.contrast)
        image = ImageEnhance.Color(image).enhance(settings.saturation)
        result = io.BytesIO()
        image.save(result, format="JPEG", quality=92, subsampling=0)
        return result.getvalue()


def default_devices():
    return {name: f"/dev/cam_{name}" for name in CAMERA_NAMES}


class Cameras:
    def __init__(
        self,
        devices=None,
        width=1920,
        height=1080,
        fps=30,
        fourcc="MJPG",
        out_width=1280,
        quality=85,
        encode_hz=5.0,
    ):
        devices = default_devices() if devices is None else devices
        unknown = set(devices) - set(CAMERA_NAMES)
        if unknown:
            raise ValueError(f"unknown cameras {sorted(unknown)}; expected {CAMERA_NAMES}")
        # Priority order regardless of how the caller listed them.
        self.devices = {name: devices[name] for name in CAMERA_NAMES if name in devices}
        self.width, self.height, self.fps = width, height, fps
        self.fourcc, self.out_width, self.quality = fourcc, out_width, quality
        self.encode_period = 1.0 / encode_hz
        self.frames = {}
        self.errors = {}
        self.cache = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.threads = []

    # ---- capture -----------------------------------------------------------------------------
    def _open(self, cv2, device):
        if device == "test":
            return None
        source = int(device) if str(device).isdigit() else device
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            raise OSError(f"cannot_open {device}")
        return cap

    def _test_frame(self, np, cv2, name):
        image = np.zeros((self.height, self.width, 3), np.uint8)
        image[:] = (60, 90, 60)
        image[: self.height // 2] = (200, 160, 120)
        cv2.putText(image, f"{name} {time.strftime('%H:%M:%S')}", (40, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 6)
        time.sleep(1.0 / self.fps)
        return image

    def _encode(self, cv2, name, image, captured_at, key):
        h, w = image.shape[:2]
        if self.out_width and w > self.out_width:
            image = cv2.resize(image, (self.out_width, round(h * self.out_width / w)),
                               interpolation=cv2.INTER_AREA)
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise ValueError("jpeg_encode_failed")
        with self.lock:
            self.frames[name] = Frame(key, captured_at, buffer.tobytes())
            self.errors.pop(name, None)

    def _run(self, name, device):
        try:
            import cv2
            import numpy as np
        except ImportError as error:
            with self.lock:
                self.errors[name] = f"opencv_missing: {error}"
            return
        backoff = 0.5
        while not self.stop.is_set():
            generation, count, last_encoded, delivered = uuid4().hex[:8], 0, 0.0, False
            cap = None
            try:
                cap = self._open(cv2, device)
                while not self.stop.is_set():
                    if cap is None:
                        image = self._test_frame(np, cv2, name)
                    elif not cap.grab():
                        raise OSError("camera_read_failed" if delivered else NO_FRAMES_HINT)
                    else:
                        image = None
                    # Read time stands in for exposure time; true capture is ~1 frame earlier.
                    now = camera_clock()
                    if now - last_encoded >= self.encode_period:
                        if image is None:
                            ok, image = cap.retrieve()
                            if not ok or image is None:
                                raise OSError("camera_decode_failed")
                        count += 1
                        self._encode(cv2, name, image, now, f"{name}-{generation}-{count}")
                        last_encoded = now
                        delivered = True
                        backoff = 0.5
            except Exception as error:
                with self.lock:
                    self.errors[name] = str(error)
                self.stop.wait(backoff)
                backoff = min(backoff * 2, 10.0)
            finally:
                if cap is not None:
                    cap.release()

    # ---- producer interface ------------------------------------------------------------------
    def start(self):
        for index, (name, device) in enumerate(self.devices.items()):
            if index:
                self.stop.wait(START_STAGGER_S)  # front claims USB bandwidth first
            thread = threading.Thread(target=self._run, args=(name, device),
                                      name=f"camera-{name}", daemon=True)
            thread.start()
            self.threads.append(thread)

    def put(self, camera, frame):
        """Also used by deterministic camera adapters in tests."""
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
        for thread in self.threads:
            thread.join(timeout=2)

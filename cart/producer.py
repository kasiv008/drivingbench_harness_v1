"""Cart producer: the unchanged harness HTTP producer with a USB camera instead of VisionIPC."""

import argparse
import json
from pathlib import Path

import uvicorn
from drivingbench.device import service
from drivingbench.device.ipc import NativeClient
from drivingbench.device.settings import atomic_json
from drivingbench.shared.contracts import Settings
from usb_camera import USBCameras

ROOT = Path.home() / "drivingbench-cart"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", type=Path, default=ROOT / "state")
    parser.add_argument("--native-socket", type=Path, default=ROOT / "native.sock")
    parser.add_argument("--port", type=int, default=8876)
    parser.add_argument("--settings-file", type=Path,
                        default=Path(__file__).with_name("cart_settings.json"))
    parser.add_argument("--camera", default="/dev/video0", help='device path, index, or "test"')
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--out-width", type=int, default=1280)
    parser.add_argument("--encode-hz", type=float, default=5.0)
    args = parser.parse_args()

    # Seed settings once. Later changes go through the UI (PATCH /settings) and persist.
    target = args.state_dir / "settings.json"
    if args.settings_file and args.settings_file.exists() and not target.exists():
        saved = json.loads(args.settings_file.read_text())
        value = Settings.model_validate(saved.get("settings", saved))
        target.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(target, {"settings": value.model_dump(), "revision": 0})

    cameras = USBCameras(args.camera, args.width, args.height, args.fps, args.fourcc,
                         args.out_width, encode_hz=args.encode_hz)
    app = service.create_app(args.state_dir, native=NativeClient(args.native_socket),
                             cameras=cameras, release="cart-0.1")
    uvicorn.run(app, host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()

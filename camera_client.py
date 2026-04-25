#!/usr/bin/env python3
"""
MASt3R Camera Client for Raspberry Pi Camera Module 3

Captures images from Raspberry Pi Camera Module 3, converts to PNG,
and uploads to MASt3R-SLAM server via HTTP POST.

Usage:
    python camera_client.py
    python camera_client.py --host linux-2 --port 5050 --fps 1
"""

import argparse
import io
import logging
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from downward_detector import (
    CameraDownDetector,
    CameraDownWriter,
    SamplingDownReader,
    LogSampler,
    RatioSampler,
)
import numpy as np

import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
from adafruit_lsm6ds import AccelRange, GyroRange, Rate

from PIL import Image
try:
    lanczos = Image.Resampling.LANCZOS
except AttributeError:
    lanczos = Image.LANCZOS

try:
    from picamera2 import Picamera2
except ImportError:
    print("ERROR: picamera2 not found. Install with: sudo apt install -y python3-picamera2")
    sys.exit(1)

try:
    import requests
except ImportError:
    print("ERROR: requests not found. Install with: pip install requests")
    sys.exit(1)


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Strategy selection — change these two lines to swap between samplers.
#
# LogSampler   → query(position=0.5)  — state at the midpoint of the capture
# RatioSampler → query(threshold=0.5) — majority vote over the whole capture
# ---------------------------------------------------------------------------
SAMPLER_CLASS        = LogSampler           # ← swap to RatioSampler here
SAMPLER_QUERY_KWARGS = {"position": 0.5}    # ← swap to {"threshold": 0.5}
# ---------------------------------------------------------------------------


def is_pi_4b() -> bool:
    try:
        with open('/proc/device-tree/model', 'r') as f:
            return "Raspberry Pi 4 Model B" in f.read()
    except FileNotFoundError:
        return False


class CameraClient:
    """Client for capturing and uploading camera images."""

    def __init__(self, host: str, port: int, fps: float = 1.0, save_local: bool = False):
        self.host = host
        self.port = port
        self.fps = fps
        self.interval = 1.0 / fps
        self.save_local = save_local
        self.upload_url = f"http://{host}:{port}/upload"
        self.master = is_pi_4b()

        self._upload_executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="upload")

        # One sampler instance, reused across all frames.
        self.sampler = SAMPLER_CLASS()

        if self.master:
            logger.info("Master mode (Pi 4B) — running IMU, writing GPIO")
            self.down_detector = CameraDownDetector(
                facing_down_threshold_deg=50.0,
                accel_correction_gain=0.02,
                accel_trust_tolerance=1.0,
            )
            i2c = board.I2C()
            self.sensor = ISM330DHCX(i2c)
            self.sensor.gyro_range = GyroRange.RANGE_4000_DPS
            self.sensor.accelerometer_range = AccelRange.RANGE_2G
            self.down_detector.initialize_from_stationary(self.sensor.acceleration)
            self.gyro_bias = self.down_detector.calibrate_gyro_bias(self.sensor)

            # Writer feeds sampler.notify(cameras[0]) on every IMU tick.
            self.down_writer = CameraDownWriter(
                self.down_detector,
                self.sensor,
                self.gyro_bias,
                pin1=5,
                pin2=6,
                poll_hz=200,
                sampler=self.sampler,
            )
        else:
            logger.info("Slave mode — polling GPIO input, no IMU")
            # Reader feeds sampler.notify(pin_state) at 200 Hz in a background thread.
            # Pin 5 = camera 1's output on the master; adjust for camera 2 if needed.
            self.down_reader = SamplingDownReader(pin=5, sampler=self.sampler, poll_hz=200)

        if self.save_local:
            self.save_dir = Path("captured_images")
            self.save_dir.mkdir(exist_ok=True)

        logger.info("Initializing camera...")
        try:
            self.camera = Picamera2()
            config = self.camera.create_still_configuration(
                main={"size": (2304, 1296)},
                buffer_count=2,
            )
            self.camera.configure(config)
            self.camera.controls.ExposureTime = 4000
            self.camera.start()
            time.sleep(2)
            logger.info(f"Camera ready — resolution {config['main']['size']}")
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            raise

        self._test_connection()

    def _test_connection(self):
        try:
            response = requests.get(f"http://{self.host}:{self.port}/status", timeout=5)
            if response.status_code == 200:
                logger.info(f"✓ Server at {self.host}:{self.port} — {response.json()}")
            else:
                logger.warning(f"Server returned {response.status_code}")
        except requests.exceptions.RequestException as e:
            logger.warning(f"⚠ Cannot reach server ({e}), will try anyway")

    def _downsample(self, jpeg_bytes: bytes) -> bytes:
        img = Image.open(io.BytesIO(jpeg_bytes))
        new_size = (512, int(img.height * 512 / img.width))
        buf = io.BytesIO()
        img.resize(new_size, lanczos).save(buf, format='JPEG')
        return buf.getvalue()

    def capture_and_convert(self) -> tuple[bytes, str]:
        jpeg_buffer = io.BytesIO()
        self.camera.capture_file(jpeg_buffer, format='jpeg')
        jpeg_bytes = self._downsample(jpeg_buffer.getvalue())
        filename = f"raspi_cam_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg"
        return jpeg_bytes, filename

    # ------------------------------------------------------------------
    # Sampler-wrapped capture — identical call site for master and slave
    # ------------------------------------------------------------------

    def capture_with_down_check(self) -> tuple[bytes, str, bool]:
        """
        Open the sampler window, capture, close the window, return verdict.

        The background thread (IMU on master, GPIO poll on slave) feeds
        sampler.notify() at 200 Hz throughout the capture automatically.
        No polling is needed here — just bookend the capture with
        start() / stop() and read the result.

        Returns
        -------
        (jpeg_bytes, filename, cam_down)
        """
        self.sampler.start()
        jpeg_bytes, filename = self.capture_and_convert()
        self.sampler.stop()

        cam_down = self.sampler.query(**SAMPLER_QUERY_KWARGS)

        logger.debug(
            f"Window: {self.sampler.sample_count} ticks, "
            f"down={self.sampler.down_fraction():.2f}, verdict={cam_down}"
        )
        return jpeg_bytes, filename, cam_down

    # ------------------------------------------------------------------
    # Upload
    # ------------------------------------------------------------------

    def _do_upload(self, jpeg_bytes: bytes, filename: str) -> bool:
        try:
            response = requests.post(
                self.upload_url,
                files={'file': (filename, jpeg_bytes, 'image/jpeg')},
                timeout=30,
            )
            if response.status_code == 200:
                data = response.json()
                logger.info(f"✓ {filename} (total: {data.get('total_images', '?')})")
                return True
            logger.error(f"✗ {response.status_code}: {response.text}")
            return False
        except requests.exceptions.RequestException as e:
            logger.error(f"✗ Upload error: {e}")
            return False

    def upload_image(self, jpeg_bytes: bytes, filename: str):
        future = self._upload_executor.submit(self._do_upload, jpeg_bytes, filename)
        future.add_done_callback(
            lambda f: logger.warning(f"Upload failed for {filename}")
            if not f.exception() and not f.result() else None
        )

    def save_local_copy(self, jpeg_bytes: bytes, filename: str):
        path = self.save_dir / filename
        path.write_bytes(jpeg_bytes)
        logger.debug(f"Saved {path}")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        logger.info(f"Capture loop: {self.fps} FPS → {self.upload_url}")
        logger.info(f"Sampler: {SAMPLER_CLASS.__name__}, query kwargs: {SAMPLER_QUERY_KWARGS}")
        logger.info("Press Ctrl+C to stop")

        frame_count = 0
        try:
            while True:
                start_time = time.time()
                try:
                    jpeg_bytes, filename, cam_down = self.capture_with_down_check()

                    if cam_down:
                        logger.info("Camera facing down during capture")
                        filename = filename.replace(".jpg", "_down.jpg")

                    frame_count += 1
                    capture_time = time.time() - start_time

                    if self.save_local:
                        self.save_local_copy(jpeg_bytes, filename)

                    self.upload_image(jpeg_bytes, filename)
                    submit_time = time.time() - start_time - capture_time

                except Exception as e:
                    logger.error(f"Frame error: {e}")
                    continue

                elapsed = time.time() - start_time
                print(
                    f"Frame {frame_count}: capture={capture_time:.3f}s "
                    f"submit={submit_time:.4f}s size={len(jpeg_bytes)}B down={cam_down}"
                )

                sleep_time = max(0, self.interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.warning(f"Frame took {elapsed:.2f}s > interval {self.interval:.2f}s")

        except KeyboardInterrupt:
            logger.info(f"\nStopped after {frame_count} frames")
        finally:
            self.cleanup()

    def cleanup(self):
        logger.info("Cleaning up...")
        if self.master:
            self.down_writer.stop()
        else:
            self.down_reader.stop()
        self._upload_executor.shutdown(wait=True)
        if hasattr(self, 'camera'):
            self.camera.stop()
            self.camera.close()
        logger.info("Done")


def main():
    parser = argparse.ArgumentParser(description="MASt3R Camera Client")
    parser.add_argument('--host', default='linux-2')
    parser.add_argument('--port', type=int, default=5050)
    parser.add_argument('--fps', type=float, default=1.0)
    parser.add_argument('--save-local', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        CameraClient(host=args.host, port=args.port, fps=args.fps, save_local=args.save_local).run()
    except Exception as e:
        logger.error(f"Fatal: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
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
from downward_detector import CameraDownDetector, CameraDownWriter, CameraDownReader
import numpy as np

import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
from adafruit_lsm6ds import AccelRange, GyroRange, Rate

from PIL import Image
import io

try:
    from picamera2 import Picamera2
    from picamera2.configuration import CameraConfiguration
except ImportError:
    print("ERROR: picamera2 not found. This must be run on a Raspberry Pi.")
    print("Install with: sudo apt install -y python3-picamera2")
    sys.exit(1)

try:
    import requests
except ImportError:
    print("ERROR: requests not found. Install with: pip install requests")
    sys.exit(1)


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def is_pi_4b():
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
            return "Raspberry Pi 4 Model B" in model
    except FileNotFoundError:
        return False

class CameraClient:
    """Client for capturing and uploading camera images."""

    def __init__(self, host: str, port: int, fps: float = 1.0, save_local: bool = False):
        """
        Initialize camera client.

        Args:
            host: Hostname or IP of MASt3R-SLAM server
            port: Port number of MASt3R-SLAM server
            fps: Frames per second to capture (default: 1.0)
            save_local: Save images locally as well (default: False)
        """
        self.host = host
        self.port = port
        self.fps = fps
        self.interval = 1.0 / fps
        self.save_local = save_local
        self.upload_url = f"http://{host}:{port}/upload"
        self.master = False 

        if is_pi_4b():
            self.master = True
            print("Running on Raspberry Pi 4B - enabling master mode with IMU integration")

        # Thread pool for fire-and-forget uploads (bounded to avoid unbounded
        # queue growth if the network is slower than the capture rate).
        self._upload_executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="upload")

        if self.master:
            self.down_detector = CameraDownDetector(
                facing_down_threshold_deg=50.0,
                accel_correction_gain=0.02,
                accel_trust_tolerance=1.0,
            )
            self.down_writer = CameraDownWriter(self.down_detector, 5, 6)

            i2c = board.I2C()
            self.sensor = ISM330DHCX(i2c)

            self.sensor.gyro_range = GyroRange.RANGE_4000_DPS
            self.sensor.accelerometer_range = AccelRange.RANGE_2G

            self.down_detector.initialize_from_stationary(self.sensor.acceleration)
            self.last_imu_time = time.monotonic()
            self.gyro_bias = self.down_detector.calibrate_gyro_bias(self.sensor)
        else:
            self.down_reader = CameraDownReader()

        # Create local save directory if needed
        if self.save_local:
            self.save_dir = Path("captured_images")
            self.save_dir.mkdir(exist_ok=True)

        # Initialize camera
        logger.info("Initializing Raspberry Pi Camera Module 3...")
        try:
            self.camera = Picamera2()

            config = self.camera.create_still_configuration(
                main={"size": (2304, 1296)},
                buffer_count=2
            )
            self.camera.configure(config)
            self.camera.controls.ExposureTime = 4000  # microseconds
            self.camera.start()

            time.sleep(2)

            logger.info("Camera initialized successfully")
            logger.info(f"Resolution: {config['main']['size']}")

        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            raise

        self._test_connection()

    def _test_connection(self):
        """Test connection to MASt3R-SLAM server."""
        try:
            test_url = f"http://{self.host}:{self.port}/status"
            response = requests.get(test_url, timeout=5)
            if response.status_code == 200:
                logger.info(f"✓ Connected to MASt3R-SLAM server at {self.host}:{self.port}")
                logger.info(f"Server status: {response.json()}")
            else:
                logger.warning(f"Server returned status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            logger.error(f"⚠ Cannot connect to server at {self.host}:{self.port}")
            logger.error(f"Error: {e}")
            logger.warning("Will attempt to upload anyway...")

    def _downsample(self, jpeg_bytes):
        """
        Downsample to 512x288
        """
        img = Image.open(io.BytesIO(jpeg_bytes))
        scale = 512 / img.width

        new_size = (
            int(img.width * scale),
            int(img.height * scale),
        )

        img_small = img.resize(new_size, Image.Resampling.LANCZOS)

        buf = io.BytesIO()
        img_small.save(buf, format='JPEG')
        return buf.getvalue()

    def capture_and_convert(self) -> tuple[bytes, str]:
        """
        Capture image from camera as JPEG (server will convert to PNG).

        Returns:
            Tuple of (jpeg_bytes, filename)
        """
        jpeg_buffer = io.BytesIO()
        self.camera.capture_file(jpeg_buffer, format='jpeg')
        jpeg_bytes = jpeg_buffer.getvalue()

        jpeg_bytes = self._downsample(jpeg_bytes)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"raspi_cam_{timestamp}.jpg"

        return jpeg_bytes, filename

    # ------------------------------------------------------------------
    # Fire-and-forget upload
    # ------------------------------------------------------------------

    def _do_upload(self, jpeg_bytes: bytes, filename: str) -> bool:
        """
        Perform the actual HTTP POST.  Runs in a worker thread — never call
        directly from the main loop.

        Returns:
            True if upload successful, False otherwise
        """
        try:
            files = {'file': (filename, jpeg_bytes, 'image/jpeg')}
            response = requests.post(self.upload_url, files=files, timeout=30)

            if response.status_code == 200:
                data = response.json()
                logger.info(f"✓ Uploaded: {filename} (Total images: {data.get('total_images', '?')})")
                return True
            else:
                logger.error(f"✗ Upload failed: {response.status_code}")
                logger.error(f"Response: {response.text}")
                return False

        except requests.exceptions.RequestException as e:
            logger.error(f"✗ Upload failed: {e}")
            return False

    def _upload_callback(self, future, filename: str):
        """Called automatically when an upload future completes."""
        try:
            success = future.result()
            if not success:
                logger.warning(f"Upload reported failure for {filename}")
        except Exception as e:
            logger.error(f"Upload raised an exception for {filename}: {e}")

    def upload_image(self, jpeg_bytes: bytes, filename: str):
        """
        Submit an upload task to the thread pool and return immediately.
        The upload runs in the background; results are logged via callback.

        Args:
            jpeg_bytes: JPEG image as bytes
            filename: Filename for the image
        """
        future = self._upload_executor.submit(self._do_upload, jpeg_bytes, filename)
        future.add_done_callback(lambda f: self._upload_callback(f, filename))

    # ------------------------------------------------------------------

    def save_local_copy(self, jpeg_bytes: bytes, filename: str):
        """Save local copy of image."""
        save_path = self.save_dir / filename
        with open(save_path, 'wb') as f:
            f.write(jpeg_bytes)
        logger.debug(f"Saved local copy: {save_path}")

    def run(self):
        """Main capture and upload loop."""
        logger.info(f"Starting capture loop at {self.fps} FPS")
        logger.info(f"Uploading to: {self.upload_url}")
        logger.info("Press Ctrl+C to stop")

        frame_count = 0

        try:
            while True:
                start_time = time.time()

                downward = False
                if self.master:
                    corrected_gyro = tuple(np.array(self.sensor.gyro) - self.gyro_bias)
                    cur_time = time.monotonic()
                    cameras = self.down_writer.update_and_write(self.sensor.acceleration, corrected_gyro, cur_time - self.last_imu_time)
                    self.last_imu_time = cur_time
                    downward = cameras[0]
                else:
                    self.down_reader = CameraDownReader()
                    downward = self.down_reader.read()

                if downward:
                    print("Camera facing downward, skipping capture")
                    continue

                try:
                    jpeg_bytes, filename = self.capture_and_convert()
                    frame_count += 1

                    capture_time = time.time() - start_time

                    if self.save_local:
                        self.save_local_copy(jpeg_bytes, filename)

                    # Submit upload — returns immediately, worker runs in background
                    self.upload_image(jpeg_bytes, filename)

                    submit_time = time.time() - start_time - capture_time

                except Exception as e:
                    logger.error(f"Error processing frame: {e}")
                    continue

                elapsed = time.time() - start_time
                print(
                    f"Frame {frame_count}: capture={capture_time:.3f}s, "
                    f"submit={submit_time:.4f}s, size={len(jpeg_bytes)}B"
                )

                sleep_time = max(0, self.interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.warning(
                        f"Frame processing took {elapsed:.2f}s, "
                        f"longer than interval {self.interval:.2f}s"
                    )

        except KeyboardInterrupt:
            logger.info("\nStopping camera client...")
            logger.info(f"Total frames captured: {frame_count}")

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up camera and thread-pool resources."""
        logger.info("Cleaning up...")

        # Drain any in-flight uploads before closing everything else.
        logger.info("Waiting for in-flight uploads to finish...")
        self._upload_executor.shutdown(wait=True)
        logger.info("Upload executor shut down")

        if hasattr(self, 'camera'):
            self.camera.stop()
            self.camera.close()

        logger.info("Camera client stopped")


def main():
    parser = argparse.ArgumentParser(
        description="MASt3R Camera Client for Raspberry Pi Camera Module 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default settings (1 FPS to linux-2:5050)
  python camera_client.py

  # Custom host and port
  python camera_client.py --host 192.168.1.100 --port 5050

  # Faster capture rate (2 FPS)
  python camera_client.py --fps 2

  # Save local copies
  python camera_client.py --save-local

  # Verbose logging
  python camera_client.py --verbose
        """
    )

    parser.add_argument('--host', default='linux-2',
                        help='Hostname or IP of MASt3R-SLAM server (default: linux-2)')
    parser.add_argument('--port', type=int, default=5050,
                        help='Port of MASt3R-SLAM server (default: 5050)')
    parser.add_argument('--fps', type=float, default=1.0,
                        help='Frames per second to capture (default: 1.0)')
    parser.add_argument('--save-local', action='store_true',
                        help='Save images locally as well')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    try:
        client = CameraClient(
            host=args.host,
            port=args.port,
            fps=args.fps,
            save_local=args.save_local,
        )
        client.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

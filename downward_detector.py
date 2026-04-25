"""
camera_down_detector.py
-----------------------
Determines whether any of 3 cameras (mounted on a rolling ball) is facing
downward, using raw accelerometer + gyroscope data from an IMU also mounted
on the ball.

Algorithm
---------
Orientation is tracked as a rotation matrix R (3x3, SO(3)) that maps vectors
from the IMU body frame into the world frame (world = z-up, gravity = -z).

Each timestep:
  1. Gyroscope integration  — rotate R by the measured angular velocity using
     a first-order Rodrigues step.  Fast but drifts over time.
  2. Accelerometer correction — when ||accel|| ≈ g (little linear acceleration),
     the accel vector is a noisy measurement of gravity in the body frame.
     We slerp R a small amount toward the orientation implied by that gravity
     vector.  Slow but drift-free.

Camera facing directions are stored as unit vectors in the IMU body frame.
Each step we rotate them into the world frame and check whether they point
close enough to world-down = (0, 0, -1).

Capture-window sampling
-----------------------
Both the master and slave Pis can determine whether their camera was facing
down *during* a capture, not just at the edges.

Master
    CameraDownWriter runs the IMU loop in a background thread and writes
    results to GPIO output pins.  Pass it a CaptureWindowSampler and it will
    call sampler.notify(is_down) on every tick for camera 0 (its own camera).

Slave
    SamplingDownReader runs a GPIO polling loop in a background thread,
    reading its input pin at a configurable rate.  Pass it a
    CaptureWindowSampler and it will call sampler.notify(is_down) on every
    tick.  The slave has no IMU — it trusts the master's GPIO output entirely.

Both strategies share the same CaptureWindowSampler interface:

    sampler.start()          # open window just before capture
    ...camera capture...
    sampler.stop()           # close window just after capture
    down = sampler.query()   # True / False

LogSampler   — records every tick into an in-memory list; query(position=0.5)
               returns the reading at the given fractional position in the
               window (0.0 = first tick, 1.0 = last, 0.5 = midpoint).

RatioSampler — counts total ticks and "down" ticks; query(threshold=0.5)
               returns True when down / total > threshold.

Swap strategies by changing SAMPLER_CLASS in camera_client.py.

Configuration
-------------
CAMERA_DIRECTIONS_BODY : list of 3 unit vectors (body frame)
FACING_DOWN_THRESHOLD_DEG : float
ACCEL_CORRECTION_GAIN : float  (0 < gain < 1)
ACCEL_TRUST_TOLERANCE : float  (m/s²)
"""

import abc
import numpy as np
from typing import Tuple, List, Optional
import RPi.GPIO as GPIO
import threading
import time


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CAMERA_DIRECTIONS_BODY: List[Tuple[float, float, float]] = [
    (-0.000235, 0.999222, 0.039429),  # Camera 0
    (0.454040, -0.548987, 0.701755),  # Camera 1
    (-0.411323, -0.433764, -0.801662),  # Camera 2
]

FACING_DOWN_THRESHOLD_DEG: float = 20.0
ACCEL_CORRECTION_GAIN: float = 0.02
ACCEL_TRUST_TOLERANCE: float = 1.0
G: float = 9.80665


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _rodrigues(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = _normalize(axis)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    t = 1.0 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])


def _rotation_from_gravity(accel_body: np.ndarray) -> np.ndarray:
    down_body = _normalize(-accel_body)
    world_down = np.array([0.0, 0.0, -1.0])
    axis = np.cross(down_body, world_down)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(down_body, world_down)
    if sin_angle < 1e-9:
        if cos_angle > 0:
            return np.eye(3)
        else:
            perp = np.array([1.0, 0.0, 0.0])
            if abs(down_body[0]) > 0.9:
                perp = np.array([0.0, 1.0, 0.0])
            return _rodrigues(perp, np.pi)
    axis = axis / sin_angle
    angle = np.arctan2(sin_angle, cos_angle)
    return _rodrigues(axis, angle)


def _slerp_rotation(R_current: np.ndarray, R_target: np.ndarray, t: float) -> np.ndarray:
    R_rel = R_target @ R_current.T
    angle = np.arccos(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
    if abs(angle) < 1e-9:
        return R_current
    sin_a = np.sin(angle)
    axis = np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ]) / (2.0 * sin_a)
    return _rodrigues(axis, t * angle) @ R_current


# ---------------------------------------------------------------------------
# CameraDownDetector
# ---------------------------------------------------------------------------

class CameraDownDetector:
    """
    Tracks ball orientation using a gyroscope-primary complementary filter
    and reports which cameras (if any) are currently facing downward.
    """

    def __init__(
        self,
        camera_directions_body: List[Tuple[float, float, float]] = None,
        facing_down_threshold_deg: float = FACING_DOWN_THRESHOLD_DEG,
        accel_correction_gain: float = ACCEL_CORRECTION_GAIN,
        accel_trust_tolerance: float = ACCEL_TRUST_TOLERANCE,
    ):
        dirs = camera_directions_body or CAMERA_DIRECTIONS_BODY
        if len(dirs) != 3:
            raise ValueError("Exactly 3 camera directions are required.")

        self._cameras_body = np.array([_normalize(np.array(d, dtype=float)) for d in dirs])
        self._threshold_cos = np.cos(np.radians(facing_down_threshold_deg))
        self._gain = accel_correction_gain
        self._trust_tol = accel_trust_tolerance
        self._R: np.ndarray = np.eye(3)
        self._initialized: bool = False

        self.cameras_facing_down: List[bool] = [False, False, False]
        self.camera_world_directions: List[np.ndarray] = [np.zeros(3) for _ in range(3)]

    def initialize_from_stationary(self, accel: Tuple[float, float, float]) -> None:
        self._R = _rotation_from_gravity(np.array(accel, dtype=float))
        self._initialized = True

    def update(
        self,
        accel: Tuple[float, float, float],
        gyro: Tuple[float, float, float],
        dt: float,
    ) -> List[bool]:
        accel_np = np.array(accel, dtype=float)
        gyro_np  = np.array(gyro,  dtype=float)

        if not self._initialized:
            self._R = _rotation_from_gravity(accel_np)
            self._initialized = True

        angle = np.linalg.norm(gyro_np) * dt
        if angle > 0.075 * dt:
            self._R = self._R @ _rodrigues(gyro_np, angle).T

        accel_mag = np.linalg.norm(accel_np)
        if abs(accel_mag - G) < self._trust_tol:
            self._R = _slerp_rotation(self._R, _rotation_from_gravity(accel_np), self._gain)

        self._R = self._reorthogonalize(self._R)

        world_down = np.array([0.0, 0.0, -1.0])
        for i, cam_body in enumerate(self._cameras_body):
            cam_world = self._R @ cam_body
            self.camera_world_directions[i] = cam_world
            self.cameras_facing_down[i] = float(np.dot(cam_world, world_down)) >= self._threshold_cos

        return self.cameras_facing_down

    def calibrate_gyro_bias(self, imu, num_samples: int = 500) -> np.ndarray:
        samples = [imu.gyro for _ in range(num_samples)]
        bias = np.mean(samples, axis=0)
        print(f"Gyro bias: {bias}")
        return bias

    @staticmethod
    def _reorthogonalize(R: np.ndarray) -> np.ndarray:
        U, _, Vt = np.linalg.svd(R)
        return U @ Vt


# ---------------------------------------------------------------------------
# Capture-window samplers
# ---------------------------------------------------------------------------

class CaptureWindowSampler(abc.ABC):
    """
    Shared interface for capture-window down-detection.  Used identically on
    both the master Pi (fed by CameraDownWriter) and each slave Pi (fed by
    SamplingDownReader).

    The background polling thread calls notify(is_down) on every tick with a
    single bool — the down-status of *this* Pi's camera only.  Each Pi owns
    its own sampler instance; there is no shared state across Pis.

    Main thread protocol:
        sampler.start()          # just before capture begins
        ...camera is busy...
        sampler.stop()           # just after capture ends
        down = sampler.query()   # True / False
    """

    @abc.abstractmethod
    def start(self) -> None:
        """Open a new capture window, discarding any previous state."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Close the capture window and freeze state for querying."""

    @abc.abstractmethod
    def query(self, **kwargs) -> bool:
        """
        Return True if this camera was facing down during the closed window.

        Keyword arguments are strategy-specific:
            LogSampler   → position  (float, 0.0–1.0, default 0.5)
            RatioSampler → threshold (float, 0.0–1.0, default 0.5)
        """

    @abc.abstractmethod
    def notify(self, is_down: bool) -> None:
        """
        Record one tick from the background polling thread.
        Must be thread-safe and non-blocking.
        """

    @property
    @abc.abstractmethod
    def sample_count(self) -> int:
        """Number of ticks recorded in the most recently closed window."""

    @abc.abstractmethod
    def down_fraction(self) -> float:
        """Fraction of ticks where the camera was down in the closed window."""


class LogSampler(CaptureWindowSampler):
    """
    Records every tick's bool into an in-memory list.

    query(position=0.5) returns the reading at the given fractional position
    within the closed window:
        position=0.0  →  first tick recorded
        position=0.5  →  tick closest to the midpoint  (default)
        position=1.0  →  last tick recorded

    Use this when you want to know the state at a *specific moment* during
    the exposure rather than a summary across it.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._log: List[bool] = []
        self._frozen: List[bool] = []
        self._active: bool = False

    def start(self) -> None:
        with self._lock:
            self._log = []
            self._frozen = []
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._frozen = list(self._log)
            self._log = []

    def query(self, position: float = 0.5) -> bool:
        if not (0.0 <= position <= 1.0):
            raise ValueError(f"position must be in [0.0, 1.0], got {position}")
        if not self._frozen:
            return False
        idx = max(0, min(int(round(position * (len(self._frozen) - 1))),
                         len(self._frozen) - 1))
        return self._frozen[idx]

    def notify(self, is_down: bool) -> None:
        with self._lock:
            if self._active:
                self._log.append(is_down)

    @property
    def sample_count(self) -> int:
        return len(self._frozen)

    def down_fraction(self) -> float:
        if not self._frozen:
            return 0.0
        return sum(self._frozen) / len(self._frozen)


class RatioSampler(CaptureWindowSampler):
    """
    Counts total ticks and "facing down" ticks within the window.

    query(threshold=0.5) returns True when down / total > threshold.

    Use this when you want a majority-vote answer over the whole exposure.
    Costs two integers per window — no list allocation.

    Threshold guide:
        0.5  →  simple majority  (default)
        0.8  →  camera was mostly down for most of the exposure
        0.1  →  flag any significant downward period
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: bool = False
        self._total: int = 0
        self._down: int = 0
        self._frozen_total: int = 0
        self._frozen_down: int = 0

    def start(self) -> None:
        with self._lock:
            self._total = 0
            self._down = 0
            self._frozen_total = 0
            self._frozen_down = 0
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._frozen_total = self._total
            self._frozen_down = self._down

    def query(self, threshold: float = 0.5) -> bool:
        if not (0.0 <= threshold <= 1.0):
            raise ValueError(f"threshold must be in [0.0, 1.0], got {threshold}")
        if self._frozen_total == 0:
            return False
        return (self._frozen_down / self._frozen_total) > threshold

    def notify(self, is_down: bool) -> None:
        with self._lock:
            if self._active:
                self._total += 1
                if is_down:
                    self._down += 1

    @property
    def sample_count(self) -> int:
        return self._frozen_total

    def down_fraction(self) -> float:
        if self._frozen_total == 0:
            return 0.0
        return self._frozen_down / self._frozen_total


# ---------------------------------------------------------------------------
# Master: IMU writer (camera 0) + GPIO output (cameras 1, 2)
# ---------------------------------------------------------------------------

class CameraDownWriter:
    """
    Runs the IMU polling loop in a background thread.

    On every tick:
      - updates the CameraDownDetector
      - writes cameras[1] and cameras[2] to GPIO output pins for slaves
      - calls sampler.notify(cameras[0]) so the master can sample its own
        camera across the capture window

    The sampler receives a single bool (camera 0 only).  Call
    sampler.start() / stop() / query() from the main thread.
    """

    def __init__(
        self,
        detector: CameraDownDetector,
        sensor,
        gyro_bias: np.ndarray,
        pin1: int = 5,
        pin2: int = 6,
        poll_hz: int = 200,
        sampler: Optional[CaptureWindowSampler] = None,
    ):
        self.detector = detector
        self.sensor = sensor
        self.gyro_bias = gyro_bias
        self.PIN1 = pin1
        self.PIN2 = pin2
        self.poll_hz = poll_hz
        self._sampler = sampler

        self._cameras: List[bool] = [False, False, False]
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN2, GPIO.OUT, initial=GPIO.LOW)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        last_time = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            accel = self.sensor.acceleration
            corrected_gyro = tuple(np.array(self.sensor.gyro) - self.gyro_bias)
            cameras = self.detector.update(accel, corrected_gyro, dt)

            GPIO.output(self.PIN1, GPIO.HIGH if cameras[1] else GPIO.LOW)
            GPIO.output(self.PIN2, GPIO.HIGH if cameras[2] else GPIO.LOW)

            with self._lock:
                self._cameras = list(cameras)

            # Notify outside the writer lock to avoid nested locking.
            if self._sampler is not None:
                self._sampler.notify(cameras[0])

            elapsed = time.monotonic() - now
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def read(self) -> List[bool]:
        """Snapshot of the latest per-camera down-status."""
        with self._lock:
            return list(self._cameras)

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join()


# ---------------------------------------------------------------------------
# Slave: GPIO input reader + sampler feed
# ---------------------------------------------------------------------------

class SamplingDownReader:
    """
    Polls a single GPIO input pin in a background thread and feeds readings
    to a CaptureWindowSampler.

    This is the slave equivalent of CameraDownWriter.  The slave has no IMU;
    it reads the pin that the master writes for its camera and lets the
    sampler accumulate a window of readings at poll_hz.

    Parameters
    ----------
    pin     : BCM input pin (must match the master's output pin for this
              camera: pin1 for camera 1, pin2 for camera 2)
    sampler : CaptureWindowSampler — receives notify(is_down) every tick
    poll_hz : polling rate in Hz (default 200, matching the master IMU rate)

    Usage
    -----
        reader = SamplingDownReader(pin=5, sampler=sampler)
        # in the capture loop:
        sampler.start()
        jpeg_bytes, filename = capture_and_convert()
        sampler.stop()
        cam_down = sampler.query(**SAMPLER_QUERY_KWARGS)
    """

    def __init__(
        self,
        pin: int,
        sampler: CaptureWindowSampler,
        poll_hz: int = 200,
    ):
        self._pin = pin
        self._sampler = sampler
        self._poll_hz = poll_hz
        self._stop_event = threading.Event()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        interval = 1.0 / self._poll_hz
        while not self._stop_event.is_set():
            now = time.monotonic()
            self._sampler.notify(GPIO.input(self._pin) == GPIO.HIGH)
            elapsed = time.monotonic() - now
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def read(self) -> bool:
        """Single instantaneous read (unchanged from CameraDownReader)."""
        return GPIO.input(self._pin) == GPIO.HIGH

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join()


# ---------------------------------------------------------------------------
# Legacy single-shot reader
# ---------------------------------------------------------------------------

class CameraDownReader:
    """
    Single-shot GPIO reader.  Use SamplingDownReader when capture-window
    sampling is needed; keep this only for simple one-shot reads.
    """

    def __init__(self, pin: int = 5):
        self.PIN = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def read(self) -> bool:
        return GPIO.input(self.PIN) == GPIO.HIGH
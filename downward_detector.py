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

Usage
-----
1. Instantiate CameraDownDetector once (before the polling loop).
2. Instantiate DownwardFrameFilter once, wrapping the detector.
3. Call filter.update(accel, gyro, dt) each polling iteration.
4. Read filter.cameras_facing_down for the temporally-stable result.

   Alternatively, use CameraDownDetector directly (without the filter) and
   read detector.cameras_facing_down for raw per-frame results.

Configuration
-------------
CAMERA_DIRECTIONS_BODY : list of 3 unit vectors (body frame)
    Replace the default XY-plane vectors with your physically measured ones.
    Each vector must be a unit vector pointing in the direction the camera faces.

FACING_DOWN_THRESHOLD_DEG : float
    A camera is "facing down" when the angle between its world-frame direction
    and the world-down vector (0,0,-1) is less than this value.

ACCEL_CORRECTION_GAIN : float  (0 < gain < 1)
    How aggressively the accelerometer corrects gyro drift each step.
    Lower = trust gyro more (better during fast rolling).
    Higher = track gravity faster (better when slow).

ACCEL_TRUST_TOLERANCE : float  (m/s²)
    Accelerometer correction is only applied when
    | ||accel|| - g | < ACCEL_TRUST_TOLERANCE.
    Keeps correction disabled during strong linear / centripetal acceleration.

SUSTAINED_FRAMES_REQUIRED : int
    Number of consecutive frames a camera must be facing down before
    DownwardFrameFilter reports it as truly facing down.  Raising this value
    reduces false positives at the cost of increased detection latency.
"""

import numpy as np
from typing import Tuple, List
import RPi.GPIO as GPIO


# ---------------------------------------------------------------------------
# Configuration — edit these to match your physical setup
# ---------------------------------------------------------------------------

# Camera facing directions as unit vectors in the IMU body frame.
# Default: 3 cameras evenly spaced (120° apart) in the IMU XY-plane.
# Replace with your measured vectors; they do NOT need to lie on a major plane.
CAMERA_DIRECTIONS_BODY: List[Tuple[float, float, float]] = [
    (0.005076, 0.997434, 0.071414),  # Camera 0
    (0.434201, -0.586046, 0.684120),  # Camera 1
    (-0.434202, -0.400043, -0.807115),  # Camera 2
]

# Angular tolerance: a camera is "facing down" if it is within this many
# degrees of the world-down direction (0, 0, -1).
FACING_DOWN_THRESHOLD_DEG: float = 20.0

# Complementary filter gains
ACCEL_CORRECTION_GAIN: float = 0.02   # per-step slerp weight toward accel estimate
ACCEL_TRUST_TOLERANCE: float = 1.0    # m/s² — how close to 9.81 to trust accel

# Standard gravity
G: float = 9.80665  # m/s²

# Number of consecutive frames a camera must face down before
# DownwardFrameFilter reports it as truly facing down.
SUSTAINED_FRAMES_REQUIRED: int = 5


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _normalize(v: np.ndarray) -> np.ndarray:
    """Return the unit vector of v, or v unchanged if it is near-zero."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _rodrigues(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """
    Rodrigues' rotation formula: 3x3 rotation matrix for rotating
    `angle_rad` radians about unit vector `axis`.
    """
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
    """
    Compute a rotation matrix R such that R @ accel_body_normalized ≈ (0,0,-1)
    (world-down direction).  The yaw component is unconstrained and set to 0.

    This is used only for initialization and for the accelerometer correction
    direction — it does NOT directly overwrite R; we slerp toward it.
    """
    down_body = _normalize(-accel_body)       # gravity pulls down; accel reads up
    world_down = np.array([0.0, 0.0, -1.0])

    # Rotation axis and angle from down_body → world_down
    axis = np.cross(down_body, world_down)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(down_body, world_down)

    if sin_angle < 1e-9:
        # Already aligned (or exactly opposite)
        if cos_angle > 0:
            return np.eye(3)
        else:
            # 180° flip — pick an arbitrary perpendicular axis
            perp = np.array([1.0, 0.0, 0.0])
            if abs(down_body[0]) > 0.9:
                perp = np.array([0.0, 1.0, 0.0])
            return _rodrigues(perp, np.pi)

    axis = axis / sin_angle
    angle = np.arctan2(sin_angle, cos_angle)
    return _rodrigues(axis, angle)


def _slerp_rotation(R_current: np.ndarray,
                    R_target: np.ndarray,
                    t: float) -> np.ndarray:
    """
    Spherical linear interpolation between two rotation matrices.
    t=0 → R_current, t=1 → R_target.
    Implemented via the relative rotation's axis-angle.
    """
    R_rel = R_target @ R_current.T          # relative rotation
    # Extract axis-angle from R_rel
    angle = np.arccos(
        np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
    )
    if abs(angle) < 1e-9:
        return R_current
    sin_a = np.sin(angle)
    axis = np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ]) / (2.0 * sin_a)
    R_step = _rodrigues(axis, t * angle)
    return R_step @ R_current


# ---------------------------------------------------------------------------
# Main detector class
# ---------------------------------------------------------------------------

class CameraDownDetector:
    """
    Tracks ball orientation using a gyroscope-primary complementary filter
    and reports which cameras (if any) are currently facing downward.

    Parameters
    ----------
    camera_directions_body : list of 3-tuples, optional
        Unit vectors for each camera's facing direction in the IMU body frame.
        Defaults to CAMERA_DIRECTIONS_BODY defined at the top of this file.
    facing_down_threshold_deg : float, optional
        Angular tolerance in degrees for "facing down". Defaults to
        FACING_DOWN_THRESHOLD_DEG.
    accel_correction_gain : float, optional
        Per-step slerp weight for accelerometer correction (0–1).
    accel_trust_tolerance : float, optional
        m/s² window around g within which accel correction is applied.
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

        # Store camera unit vectors in body frame as (3,3) array, one row each
        self._cameras_body = np.array([
            _normalize(np.array(d, dtype=float)) for d in dirs
        ])

        self._threshold_cos = np.cos(np.radians(facing_down_threshold_deg))
        self._gain = accel_correction_gain
        self._trust_tol = accel_trust_tolerance

        # Orientation estimate: maps body → world.  Initialised to identity;
        # will be set properly on the first update() call.
        self._R: np.ndarray = np.eye(3)
        self._initialized: bool = False

        # Public outputs (updated every call to update())
        self.cameras_facing_down: List[bool] = [False, False, False]
        self.camera_world_directions: List[np.ndarray] = [
            np.zeros(3) for _ in range(3)
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def initialize_from_stationary(
        self,
        accel: Tuple[float, float, float],
    ) -> None:
        """
        Bootstrap the orientation estimate from a single stationary reading.
        Call this once before the polling loop begins, while the ball is still.

        Parameters
        ----------
        accel : (ax, ay, az) in m/s²
            Accelerometer reading while the ball is completely stationary.
        """
        accel_np = np.array(accel, dtype=float)
        self._R = _rotation_from_gravity(accel_np)
        self._initialized = True

    def update(
        self,
        accel: Tuple[float, float, float],
        gyro: Tuple[float, float, float],
        dt: float,
    ) -> List[bool]:
        """
        Process one IMU sample and update which cameras face downward.

        Parameters
        ----------
        accel : (ax, ay, az) in m/s²
            Raw accelerometer reading.
        gyro : (gx, gy, gz) in rad/s
            Raw gyroscope reading.
        dt : float
            Time elapsed since the last call, in seconds.

        Returns
        -------
        List[bool]  — same object as self.cameras_facing_down
            cameras_facing_down[i] is True if camera i is within
            facing_down_threshold_deg of straight down.
        """
        accel_np = np.array(accel, dtype=float)
        gyro_np  = np.array(gyro,  dtype=float)

        # --- Bootstrap if update() is called before initialize_from_stationary
        if not self._initialized:
            self._R = _rotation_from_gravity(accel_np)
            self._initialized = True

        # --- 1. Gyroscope integration (Rodrigues step) --------------------
        angle = np.linalg.norm(gyro_np) * dt
        if angle > 0.075 * dt:
            R_gyro = _rodrigues(gyro_np, angle)
            # Gyro is measured in body frame; rotate R from the right
            self._R = self._R @ R_gyro.T  # R_new = R_old · R_delta^T maps body→world

        # --- 2. Accelerometer correction ----------------------------------
        accel_mag = np.linalg.norm(accel_np)
        if abs(accel_mag - G) < self._trust_tol:
            R_accel = _rotation_from_gravity(accel_np)
            self._R = _slerp_rotation(self._R, R_accel, self._gain)

        # --- 3. Re-orthogonalise R (prevents numerical drift) -------------
        self._R = self._reorthogonalize(self._R)

        # --- 4. Evaluate each camera -------------------------------------
        world_down = np.array([0.0, 0.0, -1.0])
        for i, cam_body in enumerate(self._cameras_body):
            cam_world = self._R @ cam_body
            self.camera_world_directions[i] = cam_world
            # dot product with world_down: 1.0 = perfectly facing down
            dot = float(np.dot(cam_world, world_down))
            self.cameras_facing_down[i] = dot >= self._threshold_cos

        return self.cameras_facing_down

    def calibrate_gyro_bias(self, imu, num_samples=500):
        """
        Call this while the ball is completely stationary at startup.
        Returns the bias vector to subtract from all future gyro readings.
        """
        samples = []
        for _ in range(num_samples):
            samples.append(imu.gyro)
        bias = np.mean(samples, axis=0)
        print(f"Gyro bias: {bias}")
        return bias

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _reorthogonalize(R: np.ndarray) -> np.ndarray:
        """
        Project R back onto SO(3) via SVD to counter floating-point drift.
        Called every step; cheap for a 3x3 matrix.
        """
        U, _, Vt = np.linalg.svd(R)
        return U @ Vt

class CameraDownWriter:
    """Writes results to two GPIO pins for slaves to read"""
    def __init__(self, detector: CameraDownDetector, pin1=18, pin2=23):
        self.detector = detector
        self.PIN1 = pin1
        self.PIN2 = pin2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN1, GPIO.OUT)
        GPIO.setup(self.PIN2, GPIO.OUT)
    
    def update_and_write(self,
        accel: Tuple[float, float, float],
        gyro: Tuple[float, float, float],
        dt: float,):
        cameras = self.detector.update(accel, gyro, dt)

        GPIO.output(self.PIN1, GPIO.HIGH if cameras[1] else GPIO.LOW)
        GPIO.output(self.PIN2, GPIO.HIGH if cameras[2] else GPIO.LOW)
        
        return cameras


class CameraDownReader:
    """Reads results from GPIO pins written by CameraDownWriter"""
    def __init__(self, pin=5):
        self.PIN = pin  # Must match the connected pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    def read(self):
        return GPIO.input(self.PIN) == GPIO.HIGH
    
"""
Pupil Core Eye Tracking with Arduino Integration
================================================

All docstrings and print statements were written with Windsurf (AI-powered IDE)

This module provides a robust eye-tracking based interaction system that connects
the Pupil Core eye tracker with Arduino-controlled physical LEDs. The system enables
users to activate LEDs by looking at them, creating a gaze-based interface.

Technical Overview:
------------------
1. Calibration: The system collects eye tracking data while the user looks at each LED
   and builds statistical models (mean and covariance matrices) for each point.

2. Detection: During real-time detection, the system compares current gaze data to
   calibrated models using Mahalanobis distance to determine which point the user
   is looking at.

3. Filtering: Advanced filtering techniques are employed to ensure stable detection:
   - Saccade filtering: Ignores rapid eye movements
   - Temporal smoothing: Applies weighted averaging to reduce noise
   - Hysteresis: Requires multiple consecutive detections to change state
   - Minimum fixation duration: Prevents accidental activations

Hardware Requirements:
--------------------
- Pupil Core eye tracker (https://pupil-labs.com/products/core/)
- Arduino board with LEDs connected
- Computer with Pupil Service running

Software Dependencies:
--------------------
- zmq: For ZeroMQ messaging to communicate with Pupil Core
- msgpack: For message packing/unpacking
- numpy: For numerical operations
- scipy: For Mahalanobis distance calculation
- serial: For Arduino communication

Usage:
-----
1. Connect Pupil Core and start Pupil Service
2. Connect Arduino to the specified COM port
3. Run this script to start calibration and detection

Example:
```python
from calibration_n_communication import RimCalibrator

# Create calibrator with custom settings
calibrator = RimCalibrator(
    num_points=3,           # Number of LEDs
    calib_duration=3,       # Seconds per calibration point
    serial_port='COM7'      # Arduino COM port
)

# Run calibration followed by detection
calibrator.calibrate()
calibrator.run_detection()
```

Author: CS448i Computational Imaging
Version: 1.0
"""

import zmq
from msgpack import loads
import numpy as np
from collections import deque
from scipy.spatial.distance import mahalanobis
import time
import serial


class RimCalibrator:
    """
    A class for calibrating and detecting eye gaze positions relative to physical LED points.

    This class handles the calibration of multiple gaze points and provides real-time
    detection with advanced filtering to control Arduino-connected LEDs based on where
    the user is looking. It implements statistical modeling of gaze data and uses
    Mahalanobis distance for robust point detection.

    Key Features:
    - Multi-point calibration with statistical modeling
    - Saccade filtering to ignore rapid eye movements
    - Temporal smoothing for stable detection
    - Hysteresis state machine to prevent flickering
    - Minimum fixation duration to avoid accidental activations

    Example Usage:
    --------------
    ```python
    # Create calibrator with 3 points
    calibrator = RimCalibrator(num_points=3)

    # Run calibration process
    calibrator.calibrate()

    # Start real-time detection
    calibrator.run_detection()
    ```
    """

    def __init__(
        self,
        num_points=3,
        calib_duration=3,
        sample_window=0.2,
        serial_port="COM7",
        baudrate=9600,
    ):
        """
        Initialize the RimCalibrator with configuration parameters.

        Parameters:
        -----------
        num_points : int
            Number of calibration points (LEDs) to use for the system
        calib_duration : float
            Duration in seconds to collect samples for each calibration point
        sample_window : float
            Time window in seconds for collecting samples during detection phase
        serial_port : str
            COM port for Arduino connection (e.g., 'COM7' on Windows, '/dev/ttyUSB0' on Linux)
        baudrate : int
            Baud rate for Arduino serial communication (must match Arduino sketch setting)

        Notes:
        ------
        This class requires:
        1. A running Pupil Core eye tracker with the Pupil Service running
        2. An Arduino connected to the specified serial port
        3. The Arduino should be programmed to respond to single character commands:
           - '0' to 'n-1': Activate specific LED (where n is num_points)
           - 'x': Turn off all LEDs
        """
        # Basic calibration parameters
        self.num_points = num_points
        self.calib_duration = calib_duration
        self.sample_window = sample_window
        self.calibration_data = {}
        self.covariances = {}
        self.means = {}
        self.current_point = 0
        self.calibrated = False
        self.last_command = None

        # Expected eye tracking features
        self.expected_keys = [
            "eye0_theta",
            "eye0_phi",
            "eye0_norm_x",
            "eye0_norm_y",
            "eye1_theta",
            "eye1_phi",
            "eye1_norm_x",
            "eye1_norm_y",
        ]

        # Saccade filtering and hysteresis parameters
        self.SACCADE_VELOCITY_THRESH = (
            50  # Velocity threshold for saccade detection (deg/ms)
        )
        self.ACTIVATION_HYSTERESIS = (
            5  # Number of consecutive positive detections needed
        )
        self.DEACTIVATION_HYSTERESIS = (
            7  # Number of consecutive negative detections needed
        )
        self.MIN_FIXATION_DURATION = 400  # Minimum fixation duration in milliseconds
        self.state_buffer = deque(maxlen=10)  # Buffer for temporal smoothing

        # State tracking variables
        self.last_gaze_position = None
        self.last_timestamp = None
        self.activation_counter = 0
        self.deactivation_counter = 0
        self.current_state = False

        # Serial setup
        try:
            self.arduino = serial.Serial(serial_port, baudrate, timeout=1)
            self.arduino_connected = True
        except serial.SerialException as e:
            print(f"Arduino connection failed: {e}")
            self.arduino_connected = False

        # ZMQ setup
        context = zmq.Context()
        self.req = context.socket(zmq.REQ)
        self.req.connect("tcp://127.0.0.1:50020")
        self.req.send_string("SUB_PORT")
        sub_port = self.req.recv_string()

        self.sub = context.socket(zmq.SUB)
        self.sub.connect(f"tcp://127.0.0.1:{sub_port}")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "pupil.")

    def calculate_gaze_velocity(self, current_features, timestamp):
        """
        Calculate gaze velocity using finite differences.

        This method computes the rate of change of gaze position between consecutive
        frames, which is used for saccade detection to filter out rapid eye movements.
        Saccades are high-velocity eye movements that should be ignored during detection.

        Parameters:
        -----------
        current_features : dict
            Current eye tracking features
        timestamp : float
            Current timestamp in seconds

        Returns:
        --------
        float
            Calculated gaze velocity in arbitrary units (higher values indicate faster movement)
        """
        if self.last_gaze_position is None or self.last_timestamp is None:
            velocity = 0
        else:
            dt = timestamp - self.last_timestamp
            if dt == 0:
                return 0

            # Calculate Euclidean distance between current and previous positions
            pos_diff = np.linalg.norm(
                np.array(list(current_features.values()))
                - np.array(list(self.last_gaze_position.values()))
            )
            velocity = pos_diff / dt

        # Update state for next calculation
        self.last_gaze_position = current_features
        self.last_timestamp = timestamp
        return velocity

    def smooth_features(self, features_buffer):
        """
        Apply weighted temporal smoothing to eye tracking features.

        This method implements a weighted moving average that prioritizes more recent
        samples, which helps reduce noise while maintaining responsiveness to actual
        gaze changes. This is crucial for stable detection without excessive lag.

        Parameters:
        -----------
        features_buffer : deque
            Buffer of recent eye tracking feature dictionaries

        Returns:
        --------
        dict
            Smoothed feature dictionary with the same keys as input features
        """
        # Create weights that increase linearly with recency
        weights = np.arange(1, len(features_buffer) + 1)
        weighted_features = {}

        # Calculate weighted average for each feature
        for key in features_buffer[0].keys():
            values = [f[key] for f in features_buffer]
            weighted_values = np.average(values, weights=weights)
            weighted_features[key] = weighted_values

        return weighted_features

    def run_detection(self, activation_threshold=1):
        """
        Run continuous gaze detection with enhanced filtering and hysteresis.

        This method implements real-time gaze detection with several advanced features:
        1. Saccade filtering to ignore rapid eye movements
        2. Temporal smoothing for stable detection
        3. Hysteresis state machine to prevent flickering
        4. Minimum fixation duration to avoid accidental activations

        The detection process runs indefinitely until interrupted by the user (Ctrl+C).

        Parameters:
        -----------
        activation_threshold : float
            Mahalanobis distance threshold for point activation (lower = more sensitive)

        Raises:
        -------
        ValueError
            If called before calibration is complete
        """
        if not self.calibrated:
            raise ValueError("❌ System not calibrated! Please run calibrate() first.")

        # Initialize buffers and state variables
        sample_buffer = deque(maxlen=int(self.sample_window * 120))
        state_history = deque(maxlen=5)  # For state consistency checks
        last_activation_time = 0

        print("\n" + "=" * 50)
        print("👁️  STARTING ENHANCED REAL-TIME DETECTION")
        print("=" * 50)
        print(f"• Activation threshold: {activation_threshold}")
        print(f"• Saccade velocity threshold: {self.SACCADE_VELOCITY_THRESH}")
        print(
            f"• Activation hysteresis: {self.ACTIVATION_HYSTERESIS} consecutive detections"
        )
        print(
            f"• Deactivation hysteresis: {self.DEACTIVATION_HYSTERESIS} consecutive detections"
        )
        print(f"• Minimum fixation duration: {self.MIN_FIXATION_DURATION} ms")
        print("-" * 50)
        print("Press Ctrl+C to stop detection")
        print("-" * 50)

        try:
            while True:
                # Receive data from Pupil Core
                topic = self.sub.recv_string()
                msg = self.sub.recv()
                msg = loads(msg, raw=False)

                if "pupil" in topic:
                    features = self.get_features(msg)
                    if features:
                        # Calculate gaze velocity for saccade detection
                        velocity = self.calculate_gaze_velocity(features, time.time())

                        # Skip processing during saccades (rapid eye movements)
                        if velocity > self.SACCADE_VELOCITY_THRESH:
                            continue

                        # Add to sample buffer for temporal smoothing
                        sample_buffer.append(features)

                        # Process every 12th sample (adjust rate as needed)
                        if len(sample_buffer) % 12 == 0:
                            # Apply temporal smoothing for stability
                            smoothed_features = self.smooth_features(sample_buffer)

                            # Find nearest calibrated point
                            point_idx, distance = self.find_nearest_point(
                                smoothed_features
                            )

                            # Current time in milliseconds for timing calculations
                            current_time = time.time() * 1000

                            # Hysteresis state machine
                            if distance < activation_threshold:
                                # Potential activation - increment counter
                                self.activation_counter += 1
                                self.deactivation_counter = 0

                                # Check if we've had enough consecutive activations
                                # and enough time has passed since last activation
                                if (
                                    self.activation_counter
                                    >= self.ACTIVATION_HYSTERESIS
                                    and (current_time - last_activation_time)
                                    > self.MIN_FIXATION_DURATION
                                ):

                                    # Only send command if state is changing
                                    if not self.current_state:
                                        print(
                                            f"🔵 Stable activation detected at point {point_idx+1} (distance: {distance:.2f})"
                                        )
                                        self.send_arduino_command(str(point_idx))
                                        self.current_state = True
                                        last_activation_time = current_time

                                    # Reset counter after activation
                                    self.activation_counter = 0

                            else:
                                # Potential deactivation - increment counter
                                self.deactivation_counter += 1
                                self.activation_counter = 0

                                # Check if we've had enough consecutive deactivations
                                if (
                                    self.deactivation_counter
                                    >= self.DEACTIVATION_HYSTERESIS
                                    and self.current_state
                                ):
                                    print(
                                        "⚪ Stable deactivation detected - all LEDs off"
                                    )
                                    self.send_arduino_command("x")
                                    self.current_state = False
                                    self.deactivation_counter = 0

        except KeyboardInterrupt:
            # Clean shutdown on keyboard interrupt
            self.send_arduino_command("x")
            print("\n" + "-" * 50)
            print("🛑 Detection stopped by user")
            print("=" * 50)

    def send_arduino_command(self, command):
        """
        Send a command to the Arduino with error handling.

        This method sends a single character command to the Arduino to control LEDs.
        It includes error handling and prevents sending duplicate commands to reduce
        communication overhead.

        Parameters:
        -----------
        command : str
            Command to send to Arduino:
            - '0' to 'n-1': Activate specific LED (where n is num_points)
            - 'x': Turn off all LEDs

        Returns:
        --------
        bool
            True if command was sent successfully, False otherwise
        """
        if self.arduino_connected and command != self.last_command:
            try:
                self.arduino.write(command.encode("utf-8"))
                self.last_command = command
                time.sleep(0.05)  # Small delay to ensure command is processed
                print(f"📡 Sent command '{command}' to Arduino")
                return True
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                print("  Check connection and try again")
                return False
        return False

    def get_features(self, msg):
        """
        Extract eye tracking features from Pupil Core message.

        This method processes raw eye tracking data from Pupil Core and extracts
        relevant features for gaze detection. It handles various edge cases and
        ensures all expected features are present with appropriate fallback values.

        Parameters:
        -----------
        msg : dict
            Message from Pupil Core containing eye tracking data

        Returns:
        --------
        dict
            Dictionary of extracted eye tracking features with standardized keys
        """
        features = {}

        # Only process high-confidence 3D eye data
        if "3d" in msg["topic"] and msg.get("confidence", 0) > 0.8:
            eye = msg["id"]

            # Extract relevant features with fallbacks for missing data
            features.update(
                {
                    f"eye{eye}_theta": msg.get("theta", 0),
                    f"eye{eye}_phi": msg.get("phi", 0),
                    f"eye{eye}_norm_x": (
                        msg["norm_pos"][0] if len(msg.get("norm_pos", [])) > 0 else 0
                    ),
                    f"eye{eye}_norm_y": (
                        msg["norm_pos"][1] if len(msg.get("norm_pos", [])) > 1 else 0
                    ),
                }
            )

        # Ensure all expected keys are present
        for k in self.expected_keys:
            if k not in features:
                features[k] = 0

        return features

    def find_nearest_point(self, features):
        """
        Find the closest calibrated point to the current gaze position.

        This method uses Mahalanobis distance to account for the statistical distribution
        of eye tracking data for each calibration point. Mahalanobis distance is superior
        to Euclidean distance for this task because it accounts for the correlation
        between features and the variance in each dimension.

        Parameters:
        -----------
        features : dict
            Current eye tracking features

        Returns:
        --------
        tuple (int, float)
            Index of nearest point and its Mahalanobis distance

        Notes:
        ------
        A smaller Mahalanobis distance indicates a closer match to the calibrated point.
        The pseudoinverse is used for numerical stability when inverting the covariance matrix.
        """
        if not self.calibrated:
            return None, float("inf")

        min_dist = float("inf")
        best_point = None
        feature_vector = [features[k] for k in self.calibration_data[0]["keys"]]

        # Find point with minimum Mahalanobis distance
        for point_idx in self.calibration_data:
            try:
                # Calculate pseudoinverse for numerical stability
                inv_cov = np.linalg.pinv(self.covariances[point_idx])
                dist = mahalanobis(feature_vector, self.means[point_idx], inv_cov)
                if dist < min_dist:
                    min_dist = dist
                    best_point = point_idx
            except np.linalg.LinAlgError as e:
                print(f"⚠️ Linear algebra error for point {point_idx}: {e}")
                continue
            except Exception as e:
                print(f"⚠️ Error calculating distance for point {point_idx}: {e}")
                continue

        return best_point, min_dist

    def calibrate(self):
        """
        Run the calibration sequence for all points.

        This method guides the user through the calibration process for each point:
        1. Activates the corresponding LED
        2. Prompts the user to look at the LED
        3. Collects eye tracking data while the user maintains gaze
        4. Calculates statistical models (mean and covariance) for each point

        The statistical models are used later for Mahalanobis distance calculation
        during the detection phase.

        Raises:
        -------
        RuntimeError
            If insufficient data is collected for any calibration point
        """
        print("\n" + "=" * 50)
        print("🔍 STARTING CALIBRATION SEQUENCE")
        print("=" * 50)
        print(f"• Number of calibration points: {self.num_points}")
        print(f"• Calibration duration per point: {self.calib_duration} seconds")
        print(f"• Sample window: {self.sample_window} seconds")
        print("-" * 50)

        for point_idx in range(self.num_points):
            # Activate current calibration LED
            self.send_arduino_command(str(point_idx))
            input(
                f"👁️  Look at LED {point_idx+1}/{self.num_points} and press Enter to begin calibration..."
            )

            print(f"⏳ Calibrating point {point_idx+1}/{self.num_points}...")
            samples = []
            start_time = time.time()

            # Collect samples for the specified duration
            while time.time() - start_time < self.calib_duration:
                remaining = self.calib_duration - (time.time() - start_time)
                if int(remaining) % 1 == 0:
                    print(
                        f"  Collecting data... {int(remaining)} seconds remaining",
                        end="\r",
                    )

                try:
                    topic = self.sub.recv_string()
                    msg = self.sub.recv()
                    msg = loads(msg, raw=False)

                    if "pupil" in topic:
                        features = self.get_features(msg)
                        if features:
                            samples.append(features)

                except zmq.ZMQError:
                    continue

            print(" " * 60, end="\r")  # Clear the progress line

            # Calculate statistical models from collected samples
            valid_samples = [
                s for s in samples if all(v is not None for v in s.values())
            ]
            if valid_samples:
                # Convert samples to numpy array for statistical processing
                array_samples = np.array(
                    [[s[k] for k in sorted(s.keys())] for s in valid_samples]
                )
                array_samples = np.nan_to_num(array_samples)  # Handle any NaN values

                # Calculate mean and covariance for Mahalanobis distance
                self.means[point_idx] = np.mean(array_samples, axis=0)
                self.covariances[point_idx] = np.cov(
                    array_samples, rowvar=False, bias=True
                )
                self.calibration_data[point_idx] = {
                    "keys": sorted(valid_samples[0].keys()),
                    "samples": array_samples,
                }
                print(
                    f"✓ Point {point_idx+1} calibrated successfully with {len(valid_samples)} samples"
                )
            else:
                print(f"❌ Failed to collect valid samples for point {point_idx+1}")
                print("  Please ensure Pupil Core is tracking your eyes correctly")
                raise RuntimeError("Calibration failed due to insufficient data")

        # Turn off all LEDs after calibration
        self.send_arduino_command("x")
        self.calibrated = True
        print("-" * 50)
        print("✅ CALIBRATION COMPLETE!")
        print(f"• Calibrated {self.num_points} points successfully")
        print("=" * 50 + "\n")


if __name__ == "__main__":
    import argparse

    # Set up command line argument parsing
    parser = argparse.ArgumentParser(
        description="Pupil Core Eye Tracking with Arduino LED Control",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--points", type=int, default=3, help="Number of calibration points (LEDs)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Duration in seconds to collect samples for each calibration point",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=0.2,
        help="Time window in seconds for collecting samples during detection",
    )
    parser.add_argument(
        "--port", type=str, default="COM7", help="Serial port for Arduino connection"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Baud rate for Arduino serial communication",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.5,
        help="Activation threshold (Mahalanobis distance)",
    )

    # Parse arguments
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("🔍 PUPIL CORE EYE TRACKING WITH ARDUINO LED CONTROL")
    print("=" * 70)
    print("Starting with the following parameters:")
    print(f"• Number of points: {args.points}")
    print(f"• Calibration duration: {args.duration} seconds")
    print(f"• Sample window: {args.window} seconds")
    print(f"• Arduino port: {args.port}")
    print(f"• Baud rate: {args.baud}")
    print(f"• Activation threshold: {args.threshold}")
    print("-" * 70)

    try:
        # Create calibrator with specified parameters
        calibrator = RimCalibrator(
            num_points=args.points,
            calib_duration=args.duration,
            sample_window=args.window,
            serial_port=args.port,
            baudrate=args.baud,
        )

        # Run calibration followed by detection
        calibrator.calibrate()
        calibrator.run_detection(activation_threshold=args.threshold)

    except KeyboardInterrupt:
        print("\n\nProgram terminated by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("Please check your connections and try again")
    finally:
        print("\n" + "=" * 70)
        print("Session ended")
        print("=" * 70)

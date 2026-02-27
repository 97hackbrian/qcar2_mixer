#!/usr/bin/env python3
"""
QCar2 Mixer Node — Final command stage before QCar2 hardware.

Responsibilities:
  1. Mux/Mixer: selects final MotorCommands from tracked input topics
  2. Safety: checks LIDAR (±30°) for obstacles, stops robot if detected
  3. Traffic logic:
     - Semáforo + NO cebra = bypass (ignore color)
     - Semáforo rojo + cebra = stop until green
     - Solo cebra = reduce speed by configurable factor
  4. Person detection: stop until person disappears + wait timeout
  5. Stop sign detection: stop for N seconds, then advance straight for N seconds
  6. LEDs: indicate state (moving, stopped, caution, etc)

Subscribes:
  - /scan (sensor_msgs/LaserScan) — LIDAR obstacle detection
  - /detections/person (qcar2_object_detections/PersonDetection)
  - /detections/stop_sign (qcar2_object_detections/StopSignDetection)
  - /detections/traffic_light (qcar2_object_detections/TrafficLightDetection)
  - /detections/zebra_crossing (qcar2_object_detections/ZebraCrossingDetection)
  - /hybrid/motor (qcar2_interfaces/MotorCommands) — input commands

Publishes:
  - /qcar2_motor_speed_cmd (qcar2_interfaces/MotorCommands) — final output
  - /qcar2_led_cmd (qcar2_interfaces/BooleanLeds) — LED state
  - /mixer/state (std_msgs/String) — debug state info

Author: Quanser QCar2 Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from qcar2_interfaces.msg import MotorCommands, BooleanLeds

from qcar2_object_detections.msg import (
    PersonDetection,
    StopSignDetection,
    TrafficLightDetection,
    ZebraCrossingDetection,
)

import math
import numpy as np


class QCar2Mixer(Node):
    """QCar2 Mixer node for safety-aware command fusion."""

    # ── Stop sign state machine constants ─────────────────────────────────
    STOPSIGN_NONE = 0
    STOPSIGN_STOPPING = 1
    STOPSIGN_EXITING = 2

    def __init__(self):
        super().__init__('qcar2_mixer')

        # ─────────────────────────────────────────────────────────────
        # PARAMETERS
        # ─────────────────────────────────────────────────────────────
        self.declare_parameter('input_motor_topic', '/hybrid/motor')
        self.declare_parameter('output_motor_topic', '/qcar2_motor_speed_cmd')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('lidar_obstacle_distance', 0.2)  # meters
        self.declare_parameter('lidar_angle_tolerance', 30.0)   # degrees (±30°)
        self.declare_parameter('zebra_speed_reduction_factor', 0.85)  # 85% = 15% reduction
        self.declare_parameter('person_wait_timeout', 2.0)      # seconds post-person-disappear
        self.declare_parameter('stop_sign_stop_time', 0.5)      # seconds to stop at stop sign
        self.declare_parameter('stop_sign_forward_time', 0.5)   # seconds to advance straight
        self.declare_parameter('rate_hz', 50.0)

        # Detection topics
        self.declare_parameter('person_detection_topic', '/detections/person')
        self.declare_parameter('stop_sign_topic', '/detections/stop_sign')
        self.declare_parameter('traffic_light_topic', '/detections/traffic_light')
        self.declare_parameter('zebra_crossing_topic', '/detections/zebra_crossing')

        # LED configuration
        self.declare_parameter('led_output_topic', '/qcar2_led_cmd')
        self.declare_parameter('obstacle_stop_timeout', 5.0)

        # Load parameters
        self._load_parameters()

        # ─────────────────────────────────────────────────────────────
        # STATE
        # ─────────────────────────────────────────────────────────────
        self.latest_motor_cmd = None
        self.lidar_obstacle_detected = False
        self.person_detected = False
        self.stop_sign_detected = False
        self.traffic_light_detected = False
        self.traffic_light_state = 'unknown'
        self.zebra_detected = False

        self.final_speed = 0.0
        self.final_steering = 0.0
        self.stop_reason = None

        # Person timeout tracking
        self.person_last_detected_time = None
        self.person_in_wait = False

        # Stop Sign state machine
        self.stop_sign_state = self.STOPSIGN_NONE
        self.stop_sign_phase_start = None

        # ─────────────────────────────────────────────────────────────
        # SUBSCRIPTIONS
        # ─────────────────────────────────────────────────────────────
        self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self._callback_lidar,
            10
        )

        self.create_subscription(
            MotorCommands,
            self.input_motor_topic,
            self._callback_motor_cmd,
            10
        )

        self.create_subscription(
            PersonDetection,
            self.person_detection_topic,
            self._callback_person,
            10
        )

        self.create_subscription(
            StopSignDetection,
            self.stop_sign_topic,
            self._callback_stop_sign,
            10
        )

        self.create_subscription(
            TrafficLightDetection,
            self.traffic_light_topic,
            self._callback_traffic_light,
            10
        )

        self.create_subscription(
            ZebraCrossingDetection,
            self.zebra_crossing_topic,
            self._callback_zebra,
            10
        )

        # ─────────────────────────────────────────────────────────────
        # PUBLICATIONS
        # ─────────────────────────────────────────────────────────────
        self.pub_motor_cmd = self.create_publisher(
            MotorCommands,
            self.output_motor_topic,
            10
        )

        self.pub_leds = self.create_publisher(
            BooleanLeds,
            self.led_output_topic,
            10
        )

        self.pub_state = self.create_publisher(
            String,
            '/mixer/state',
            10
        )

        # ─────────────────────────────────────────────────────────────
        # TIMER (control loop)
        # ─────────────────────────────────────────────────────────────
        period = 1.0 / max(1.0, self.rate_hz)
        self.create_timer(period, self._control_loop)

        # ─────────────────────────────────────────────────────────────
        # PARAMETER CALLBACK
        # ─────────────────────────────────────────────────────────────
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info('QCar2 Mixer Node initialized.')
        self.get_logger().info(f'  Input motor topic: {self.input_motor_topic}')
        self.get_logger().info(f'  Output motor topic: {self.output_motor_topic}')
        self.get_logger().info(f'  LIDAR obstacle distance: {self.lidar_obstacle_distance} m')
        self.get_logger().info(f'  Person wait timeout: {self.person_wait_timeout} s')
        self.get_logger().info(f'  Stop sign stop time: {self.stop_sign_stop_time} s')
        self.get_logger().info(f'  Stop sign forward time: {self.stop_sign_forward_time} s')

    def _load_parameters(self):
        """Load all parameters from ROS parameter server."""
        self.input_motor_topic = self.get_parameter('input_motor_topic').value
        self.person_wait_timeout = float(
            self.get_parameter('person_wait_timeout').value
        )
        self.stop_sign_stop_time = float(
            self.get_parameter('stop_sign_stop_time').value
        )
        self.stop_sign_forward_time = float(
            self.get_parameter('stop_sign_forward_time').value
        )
        self.output_motor_topic = self.get_parameter('output_motor_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.lidar_obstacle_distance = float(
            self.get_parameter('lidar_obstacle_distance').value
        )
        self.lidar_angle_tolerance = float(
            self.get_parameter('lidar_angle_tolerance').value
        )
        self.zebra_speed_reduction_factor = float(
            self.get_parameter('zebra_speed_reduction_factor').value
        )
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.person_detection_topic = self.get_parameter('person_detection_topic').value
        self.stop_sign_topic = self.get_parameter('stop_sign_topic').value
        self.traffic_light_topic = self.get_parameter('traffic_light_topic').value
        self.zebra_crossing_topic = self.get_parameter('zebra_crossing_topic').value
        self.led_output_topic = self.get_parameter('led_output_topic').value
        self.obstacle_stop_timeout = float(
            self.get_parameter('obstacle_stop_timeout').value
        )

    def _on_params_changed(self, params):
        """Handle parameter changes at runtime."""
        for param in params:
            if param.name == 'lidar_obstacle_distance':
                self.lidar_obstacle_distance = float(param.value)
            elif param.name == 'zebra_speed_reduction_factor':
                self.zebra_speed_reduction_factor = float(param.value)
            elif param.name == 'rate_hz':
                self.rate_hz = float(param.value)
        return SetParametersResult(successful=True)

    # ─────────────────────────────────────────────────────────────
    # CALLBACKS: Detection subscribers
    # ─────────────────────────────────────────────────────────────

    def _callback_lidar(self, msg: LaserScan):
        """Process LIDAR scan for obstacles in ±30° forward cone."""
        if msg.ranges is None or len(msg.ranges) == 0:
            self.lidar_obstacle_detected = False
            return

        # Calculate angle per ray - use linspace to match exact array size
        num_ranges = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_ranges)

        # Find rays within ±angle_tolerance from 0 (forward)
        angle_tol_rad = math.radians(self.lidar_angle_tolerance)
        forward_mask = np.abs(angles) <= angle_tol_rad

        forward_ranges = np.array(msg.ranges)[forward_mask]

        # Filter out NaN/Inf and check for obstruction
        valid_ranges = forward_ranges[
            ~np.isnan(forward_ranges) & ~np.isinf(forward_ranges) & (forward_ranges > 0)
        ]

        if len(valid_ranges) > 0:
            min_range = float(np.min(valid_ranges))
            self.lidar_obstacle_detected = min_range < self.lidar_obstacle_distance
        else:
            self.lidar_obstacle_detected = False

    def _callback_motor_cmd(self, msg: MotorCommands):
        """Store latest upstream MotorCommands."""
        self.latest_motor_cmd = msg
        # Debug: confirm we're receiving motor commands
        self.get_logger().debug(
            f'Received motor cmd: {msg.motor_names} = {msg.values}',
            throttle_duration_sec=1.0
        )

    def _callback_person(self, msg: PersonDetection):
        """Track person detection with post-disappear wait."""
        if msg.detected:
            self.person_detected = True
            self.person_last_detected_time = self.get_clock().now()
            self.person_in_wait = False
        else:
            self.person_detected = False
            # Keep person_in_wait if we ever saw a person
            if self.person_last_detected_time is not None:
                self.person_in_wait = True

    def _callback_stop_sign(self, msg: StopSignDetection):
        """Trigger stop sign state machine on first detection."""
        self.stop_sign_detected = msg.detected
        if msg.detected and self.stop_sign_state == self.STOPSIGN_NONE:
            self.stop_sign_state = self.STOPSIGN_STOPPING
            self.stop_sign_phase_start = self.get_clock().now()
            self.get_logger().info('Stop sign detected — starting stop sequence.')

    def _callback_traffic_light(self, msg: TrafficLightDetection):
        """Track traffic light state."""
        self.traffic_light_detected = msg.detected
        self.traffic_light_state = msg.state if msg.detected else 'unknown'

    def _callback_zebra(self, msg: ZebraCrossingDetection):
        """Track zebra crossing."""
        self.zebra_detected = msg.detected

    # ─────────────────────────────────────────────────────────────────────────
    # CONTROL LOOP
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """Evaluate all safety/traffic conditions and publish final command."""
        # Debug heartbeat (throttled)
        has_input = self.latest_motor_cmd is not None
        self.get_logger().info(
            f'[MIXER] has_input={has_input} lidar={self.lidar_obstacle_detected} '
            f'person={self.person_detected} stopsign={self.stop_sign_state} '
            f'tl={self.traffic_light_detected} zebra={self.zebra_detected}',
            throttle_duration_sec=2.0
        )

        # If no upstream command yet, publish IDLE (stop) to ensure hardware gets commands
        if self.latest_motor_cmd is None:
            self._emit(
                steering=0.0,
                speed=0.0,
                led='orange',
                state='IDLE_NO_INPUT',
            )
            return

        now = self.get_clock().now()

        # ═══════════════════════════════════════════════════════════
        # P1 — LIDAR obstacle (highest priority)
        # ═══════════════════════════════════════════════════════════
        if self.lidar_obstacle_detected:
            self._emit(
                steering=self._steering(),
                speed=0.0,
                led='red',
                state='STOPPED_LIDAR_OBSTACLE',
            )
            return

        # ═══════════════════════════════════════════════════════════
        # P2 — Person detected
        # ═══════════════════════════════════════════════════════════
        if self.person_detected:
            self._emit(
                steering=self._steering(),
                speed=0.0,
                led='red',
                state='STOPPED_PERSON',
            )
            return

        # P2b — Post-disappear wait
        if self.person_in_wait and self.person_last_detected_time is not None:
            elapsed = (now - self.person_last_detected_time).nanoseconds * 1e-9
            if elapsed < self.person_wait_timeout:
                self._emit(
                    steering=self._steering(),
                    speed=0.0,
                    led='red',
                    state=(
                        f'PERSON_WAIT {elapsed:.1f}/{self.person_wait_timeout:.1f}s'
                    ),
                )
                return
            else:
                self.person_in_wait = False
                self.person_last_detected_time = None

        # ═══════════════════════════════════════════════════════════
        # P3 — Stop sign state machine
        # ═══════════════════════════════════════════════════════════
        if self.stop_sign_state != self.STOPSIGN_NONE:
            elapsed = (now - self.stop_sign_phase_start).nanoseconds * 1e-9

            if self.stop_sign_state == self.STOPSIGN_STOPPING:
                if elapsed < self.stop_sign_stop_time:
                    self._emit(
                        steering=self._steering(),
                        speed=0.0,
                        led='red',
                        state=(
                            f'STOPSIGN_STOP '
                            f'{elapsed:.2f}/{self.stop_sign_stop_time:.1f}s'
                        ),
                    )
                    return
                else:
                    self.stop_sign_state = self.STOPSIGN_EXITING
                    self.stop_sign_phase_start = now
                    self.get_logger().info('Stop sign — advancing straight.')
                    elapsed = 0.0

            if self.stop_sign_state == self.STOPSIGN_EXITING:
                elapsed = (now - self.stop_sign_phase_start).nanoseconds * 1e-9
                if elapsed < self.stop_sign_forward_time:
                    self._emit(
                        steering=0.0,
                        speed=self._speed(),
                        led='orange',
                        state=(
                            f'STOPSIGN_EXIT '
                            f'{elapsed:.2f}/{self.stop_sign_forward_time:.1f}s'
                        ),
                    )
                    return
                else:
                    self.stop_sign_state = self.STOPSIGN_NONE
                    self.stop_sign_phase_start = None
                    self.get_logger().info('Stop sign sequence complete.')

        # ═══════════════════════════════════════════════════════════
        # P4 — Traffic light + Zebra crossing
        # ═══════════════════════════════════════════════════════════
        if self.traffic_light_detected:
            if self.zebra_detected and self.traffic_light_state == 'RED':
                self._emit(
                    steering=self._steering(),
                    speed=0.0,
                    led='red',
                    state='STOPPED_RED_LIGHT_ZEBRA',
                )
                return
            else:
                # Any other light state (no zebra, green, yellow, unknown) → bypass
                self._emit_bypass(state='BYPASS_TRAFFIC_LIGHT')
                return

        # ═══════════════════════════════════════════════════════════
        # P5 — Only zebra crossing (no traffic light)
        # ═══════════════════════════════════════════════════════════
        if self.zebra_detected:
            reduced = self._speed() * self.zebra_speed_reduction_factor
            self._emit(
                steering=self._steering(),
                speed=reduced,
                led='orange',
                state=f'CAUTION_ZEBRA speed={reduced:.3f}m/s',
            )
            return

        # ═══════════════════════════════════════════════════════════
        # DEFAULT — bypass upstream command unchanged
        # ═══════════════════════════════════════════════════════════
        self._emit_bypass(state='BYPASS')

    # ─────────────────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _speed(self) -> float:
        """Extract motor_throttle from latest upstream command."""
        if self.latest_motor_cmd is None:
            return 0.0
        for i, name in enumerate(self.latest_motor_cmd.motor_names):
            if name == 'motor_throttle' and i < len(self.latest_motor_cmd.values):
                return float(self.latest_motor_cmd.values[i])
        return 0.0

    def _steering(self) -> float:
        """Extract steering_angle from latest upstream command."""
        if self.latest_motor_cmd is None:
            return 0.0
        for i, name in enumerate(self.latest_motor_cmd.motor_names):
            if name == 'steering_angle' and i < len(self.latest_motor_cmd.values):
                return float(self.latest_motor_cmd.values[i])
        return 0.0

    def _emit(self, steering: float, speed: float, led: str, state: str):
        """Publish motor command, LEDs, and debug state."""
        msg = MotorCommands()
        msg.motor_names = ['steering_angle', 'motor_throttle']
        msg.values = [float(steering), float(speed)]
        self.pub_motor_cmd.publish(msg)
        self._publish_leds(led)
        self._publish_state(state)
        # Debug: confirm we're publishing
        self.get_logger().debug(
            f'EMIT: steer={steering:.3f} speed={speed:.3f} state={state}',
            throttle_duration_sec=1.0
        )

    def _emit_bypass(self, state: str = 'BYPASS'):
        """Publish upstream command unchanged."""
        self._emit(
            steering=self._steering(),
            speed=self._speed(),
            led='green',
            state=state,
        )

    def _publish_leds(self, color: str):
        """Set LEDs based on control state color."""
        led_names = [
            'left_outside_brake_light',   # 0
            'left_inside_brake_light',    # 1
            'right_inside_brake_light',   # 2
            'right_outside_brake_light',  # 3
            'left_reverse_light',         # 4
            'right_reverse_light',        # 5
            'left_rear_signal',           # 6
            'right_rear_signal',          # 7
            'left_outside_headlight',     # 8
            'left_middle_headlight',      # 9
            'left_inside_headlight',      # 10
            'right_inside_headlight',     # 11
            'right_middle_headlight',     # 12
            'right_outside_headlight',    # 13
            'left_front_signal',          # 14
            'right_front_signal',         # 15
        ]
        v = [False] * 16

        if color == 'green':
            # Moving: headlights on
            v[8] = v[9] = v[10] = v[11] = v[12] = v[13] = True
        elif color == 'red':
            # Stopped: brake lights on
            v[0] = v[1] = v[2] = v[3] = True
        elif color == 'orange':
            # Caution: front signals + headlights
            v[14] = v[15] = True
            v[8] = v[9] = v[10] = v[11] = v[12] = v[13] = True

        msg = BooleanLeds()
        msg.led_names = led_names
        msg.values = v
        #self.pub_leds.publish(msg)

    def _publish_state(self, state: str):
        """Publish current mixer state string for debugging."""
        msg = String()
        msg.data = state
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QCar2Mixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

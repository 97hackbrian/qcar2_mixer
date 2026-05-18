#!/usr/bin/env python3
"""
LED Strip Controller Node — Topic-driven LED color control for QCar 2.

Subscribes to string commands and sets the LED strip color on the
real hardware by calling the `led_color_id` parameter on qcar2_hardware.

Subscribes:
  /btled              (std_msgs/String) — mission color from Behavior Tree
  /btled_override_id  (std_msgs/Int32)  — safety override (>=0 force, -1 release)

Command → Color mapping:
  "init"          →  Magenta  (led_color_id = 5)   Waiting at Taxi Hub
  "to_pickup"     →  Green    (led_color_id = 1)   Driving to pick-up
  "pickup_done"   →  Blue     (led_color_id = 2)   Pick-up completed
  "dropoff_done"  →  Orange   (led_color_id = 3)   Drop-off completed
  "idle"          →  Magenta  (led_color_id = 5)   Waiting for next ride

Hardware color IDs (from qcar2_hardware.cpp):
  0 = Red (255,0,0)  |  1 = Green (0,255,0)    |  2 = Blue (0,0,255)
  3 = Orange (255,165,0)  |  4 = Cyan (0,255,255)  |  5 = Magenta (255,0,255)

Override protocol:
  data >= 0  → force LED strip to that color ID (e.g. 0 = RED for safety)
  data == -1 → release override, restore last mission color from BT

Author: auto-generated
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rclpy.parameter import ParameterType


# ── Command → hardware color ID mapping ──────────────────────────────
#    command_string → (led_color_id, human_label)
COLOR_MAP = {
    'init':         (5, 'MAGENTA'),    # Waiting at Taxi Hub
    'to_pickup':    (1, 'GREEN'),      # Driving to pick-up
    'pickup_done':  (2, 'BLUE'),       # Pick-up completed
    'dropoff_done': (3, 'ORANGE'),     # Drop-off completed
    'idle':         (5, 'MAGENTA'),    # Waiting for next ride
}


class LEDSequenceNode(Node):
    """Receives string commands and changes the LED strip on the QCar 2."""

    def __init__(self):
        super().__init__('led_sequence_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('hardware_node_name', 'qcar2_hardware')
        self.hardware_node = self.get_parameter('hardware_node_name').value

        # ── Service client to set led_color_id on hardware node ──────
        service_name = f'/{self.hardware_node}/set_parameters'
        self.param_client = self.create_client(SetParameters, service_name)

        self.get_logger().info(f'Waiting for service {service_name} ...')
        self.param_client.wait_for_service(timeout_sec=10.0)

        if self.param_client.service_is_ready():
            self.get_logger().info(f'Connected to {service_name}')
        else:
            self.get_logger().warn(
                f'Service {service_name} not available yet — '
                'commands will be sent when it appears.'
            )

        # ── Override state for safety LED control ─────────────────────
        # last_mission_led_id: remembers the last color sent by the BT
        # override_led_id: -1 = no override, >=0 = forced color ID
        self.last_mission_led_id = 5   # default magenta (init)
        self.override_led_id = -1      # no override active

        # ── Subscriber: incoming LED commands from Behavior Tree ──────
        self.sub_cmd = self.create_subscription(
            String,
            '/btled',
            self._on_led_command,
            10,
        )

        # ── Subscriber: safety override from mixer ────────────────────
        #    data >= 0 → force that color ID on the strip
        #    data == -1 → release override, restore last BT color
        self.sub_led_override = self.create_subscription(
            Int32,
            '/btled_override_id',
            self._callback_led_override,
            10,
        )

        self.get_logger().info(
            'LED Strip Controller started  |  listening on /btled + /btled_override_id'
        )
        self.get_logger().info(
            f'Known commands: {list(COLOR_MAP.keys())}'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callback: normal BT mission color
    # ──────────────────────────────────────────────────────────────────
    def _on_led_command(self, msg: String):
        """Process an incoming LED command from the Behavior Tree.

        Always saves the color as last_mission_led_id.
        Only applies it to hardware if no safety override is active.
        """
        cmd = msg.data.strip().lower()

        # Look up the command
        entry = COLOR_MAP.get(cmd)

        if entry is None:
            self.get_logger().warn(
                f'Unknown LED command: "{msg.data}" — ignoring.  '
                f'Valid: {list(COLOR_MAP.keys())}'
            )
            return

        color_id, label = entry
        # Always remember the latest mission color from the BT
        self.last_mission_led_id = color_id

        if self.override_led_id == -1:
            # No override active → apply mission color normally
            self._set_led_color(color_id)
            self.get_logger().info(
                f'BT LED: "{cmd}" → {label} (led_color_id={color_id})'
            )
        else:
            # Override active → save color but do NOT apply it yet
            self.get_logger().info(
                f'BT LED saved but not applied (override active): '
                f'"{cmd}" → {label} id={color_id}'
            )

    # ──────────────────────────────────────────────────────────────────
    # Callback: safety override from mixer
    # ──────────────────────────────────────────────────────────────────
    def _callback_led_override(self, msg: Int32):
        """Handle LED override commands from the safety mixer.

        data >= 0 : force LED strip to that color ID (e.g. 0 = RED).
        data == -1: release override, restore last mission color.
        """
        led_id = int(msg.data)

        if led_id >= 0:
            # Activate override — force the requested color
            self.override_led_id = led_id
            self._set_led_color(led_id)
            self.get_logger().info(f'LED override ACTIVE: id={led_id}')
        else:
            # Release override — restore last BT mission color
            self.override_led_id = -1
            self._set_led_color(self.last_mission_led_id)
            self.get_logger().info(
                f'LED override RELEASED. Restoring mission id={self.last_mission_led_id}'
            )

    # ──────────────────────────────────────────────────────────────────
    # Hardware interaction
    # ──────────────────────────────────────────────────────────────────
    def _set_led_color(self, color_id: int):
        """Set led_color_id parameter on the qcar2_hardware node."""
        param = Parameter()
        param.name = 'led_color_id'
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = color_id

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.param_client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        """Log the result of the parameter-set call."""
        try:
            response = future.result()
            if response and response.results:
                for r in response.results:
                    if r.successful:
                        self.get_logger().info('LED color updated on hardware ✓')
                    else:
                        self.get_logger().warn(
                            f'Hardware rejected LED change: {r.reason}'
                        )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


# ──────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LEDSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

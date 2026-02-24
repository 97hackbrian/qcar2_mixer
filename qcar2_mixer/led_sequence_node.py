#!/usr/bin/env python3
"""
LED Strip Controller Node — Topic-driven LED color control for QCar 2.

Subscribes to string commands and sets the LED strip color on the
real hardware by calling the `led_color_id` parameter on qcar2_hardware.

Subscribes:
  /btled  (std_msgs/String)

Command → Color mapping:
  "init"          →  Magenta  (led_color_id = 5)   Waiting at Taxi Hub
  "to_pickup"     →  Green    (led_color_id = 1)   Driving to pick-up
  "pickup_done"   →  Blue     (led_color_id = 2)   Pick-up completed
  "dropoff_done"  →  Yellow   (led_color_id = 3)   Drop-off completed (closest to Orange)
  "idle"          →  Magenta  (led_color_id = 5)   Waiting for next ride

Hardware color IDs (from qcar2_hardware.cpp):
  0 = Red (255,0,0)  |  1 = Green (0,255,0)    |  2 = Blue (0,0,255)
  3 = Yellow (255,255,0)  |  4 = Cyan (0,255,255)  |  5 = Magenta (255,0,255)

Author: auto-generated
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rclpy.parameter import ParameterType


# ── Command → hardware color ID mapping ──────────────────────────────
#    command_string → (led_color_id, human_label)
COLOR_MAP = {
    'init':         (5, 'MAGENTA'),    # Waiting at Taxi Hub
    'to_pickup':    (1, 'GREEN'),      # Driving to pick-up
    'pickup_done':  (2, 'BLUE'),       # Pick-up completed
    'dropoff_done': (3, 'YELLOW'),     # Drop-off completed (closest to Orange)
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

        # ── Subscriber: incoming LED commands ─────────────────────────
        self.sub_cmd = self.create_subscription(
            String,
            '/btled',
            self._on_led_command,
            10,
        )

        self.get_logger().info(
            'LED Strip Controller started  |  listening on /btled'
        )
        self.get_logger().info(
            f'Known commands: {list(COLOR_MAP.keys())}'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callback
    # ──────────────────────────────────────────────────────────────────
    def _on_led_command(self, msg: String):
        """Process an incoming LED command and apply it to the hardware."""
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
        self.get_logger().info(
            f'LED command: "{cmd}" → {label} (led_color_id={color_id})'
        )

        # Send the parameter change to the hardware node
        self._set_led_color(color_id)

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

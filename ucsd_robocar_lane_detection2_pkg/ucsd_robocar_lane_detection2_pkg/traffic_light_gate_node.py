import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


NODE_NAME = 'traffic_light_gate_node'
DEFAULT_INPUT_CMD_TOPIC = '/cmd_vel_nav'
DEFAULT_OUTPUT_CMD_TOPIC = '/cmd_vel'
DEFAULT_STATE_TOPIC = '/traffic_light/state'
DEFAULT_DISTANCE_TOPIC = '/traffic_light/distance_m'


class TrafficLightGate(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_cmd_topic', DEFAULT_INPUT_CMD_TOPIC),
                ('output_cmd_topic', DEFAULT_OUTPUT_CMD_TOPIC),
                ('traffic_light_state_topic', DEFAULT_STATE_TOPIC),
                ('traffic_light_distance_topic', DEFAULT_DISTANCE_TOPIC),
                ('stop_on_yellow', False),
                ('stop_distance_m', 4.0),
                ('distance_tolerance_m', 0.3),
                ('distance_timeout_sec', 1.0),
            ],
        )

        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.state_topic = self.get_parameter('traffic_light_state_topic').value
        self.distance_topic = self.get_parameter('traffic_light_distance_topic').value
        self.stop_on_yellow = bool(self.get_parameter('stop_on_yellow').value)
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.distance_tolerance_m = float(self.get_parameter('distance_tolerance_m').value)
        self.distance_timeout_sec = float(self.get_parameter('distance_timeout_sec').value)

        self.allow_motion = True
        self.last_light_state = 'no_signal_yet'
        self.last_gate_status = 'unknown'
        self.last_cmd = Twist()
        self.zero_cmd = Twist()
        self.latest_distance_m = None
        self.last_distance_time = 0.0

        self.cmd_publisher = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.cmd_subscriber = self.create_subscription(Twist, self.input_cmd_topic, self.forward_cmd, 10)
        self.state_subscriber = self.create_subscription(String, self.state_topic, self.update_light_state, 10)
        self.distance_subscriber = self.create_subscription(Float32, self.distance_topic, self.update_light_distance, 10)
        self.cmd_subscriber
        self.state_subscriber
        self.distance_subscriber

        self.get_logger().info(
            f'input_cmd_topic: {self.input_cmd_topic}\n'
            f'output_cmd_topic: {self.output_cmd_topic}\n'
            f'traffic_light_state_topic: {self.state_topic}\n'
            f'traffic_light_distance_topic: {self.distance_topic}\n'
            f'stop_on_yellow: {self.stop_on_yellow}\n'
            f'stop_distance_m: {self.stop_distance_m}\n'
            f'distance_tolerance_m: {self.distance_tolerance_m}\n'
            f'distance_timeout_sec: {self.distance_timeout_sec}'
        )

    def forward_cmd(self, cmd_msg):
        self.last_cmd = cmd_msg
        if self.allow_motion:
            self.cmd_publisher.publish(cmd_msg)
        else:
            self.cmd_publisher.publish(self.zero_cmd)

    def update_light_state(self, light_msg):
        state = light_msg.data.strip().lower()
        previous_allow_motion = self.allow_motion
        self.last_light_state = state
        self.evaluate_motion()
        if previous_allow_motion and not self.allow_motion:
            self.cmd_publisher.publish(self.zero_cmd)

    def update_light_distance(self, distance_msg):
        self.latest_distance_m = float(distance_msg.data)
        self.last_distance_time = time.time()
        self.evaluate_motion()

    def has_recent_distance(self):
        if self.latest_distance_m is None:
            return False
        return (time.time() - self.last_distance_time) <= self.distance_timeout_sec

    def is_light_actionable_distance(self):
        if not self.has_recent_distance():
            return False
        return self.latest_distance_m <= self.stop_distance_m

    def should_stop_for_light(self):
        if self.last_light_state not in ('red', 'green', 'yellow'):
            return False
        if self.last_light_state == 'green':
            return False
        if not self.is_light_actionable_distance():
            return False
        if self.last_light_state == 'yellow' and not self.stop_on_yellow:
            return False
        return True

    def evaluate_motion(self):
        should_stop = self.should_stop_for_light()
        if self.last_light_state == 'green':
            self.allow_motion = True
        elif self.is_light_actionable_distance():
            self.allow_motion = not should_stop
        action = 'GO' if self.allow_motion else 'STOP'
        distance_text = 'unknown'
        if self.has_recent_distance():
            distance_text = f'{self.latest_distance_m:.2f} m'
        gate_status = f'{self.last_light_state}:{action}:{distance_text}'
        if gate_status == self.last_gate_status:
            return
        self.get_logger().info(
            f'Traffic light update: {self.last_light_state}, distance={distance_text}, '
            f'action_window<={self.stop_distance_m:.2f} m -> {action}'
        )
        self.last_gate_status = gate_status


def main(args=None):
    rclpy.init(args=args)
    gate = TrafficLightGate()
    try:
        rclpy.spin(gate)
    except KeyboardInterrupt:
        gate.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        gate.cmd_publisher.publish(gate.zero_cmd)
        time.sleep(0.2)
        gate.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

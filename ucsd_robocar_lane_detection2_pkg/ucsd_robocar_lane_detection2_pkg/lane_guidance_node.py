import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import os

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/centroid'
DEFAULT_ACTUATOR_TOPIC_NAME = '/cmd_vel_nav'
DEFAULT_LANE_MODE_TOPIC_NAME = '/lane_mode'
DEFAULT_AVOIDANCE_CMD_TOPIC_NAME = '/avoidance_cmd'
DEFAULT_CONE_DISTANCE_TOPIC_NAME = '/cone_distance_m'


class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle',0.0),
                ('max_throttle', 0.28),
                ('min_throttle', 0.24),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0),
                ('cmd_vel_output_topic', DEFAULT_ACTUATOR_TOPIC_NAME),
                ('lane_mode', 'center'),
                ('lane_mode_topic', DEFAULT_LANE_MODE_TOPIC_NAME),
                ('left_lane_target_error', 0.2),
                ('right_lane_target_error', -0.2),
                ('avoidance_cmd_topic', DEFAULT_AVOIDANCE_CMD_TOPIC_NAME),
                ('avoidance_throttle', 0.5),
                ('avoidance_left_steering', 0.6),
                ('avoidance_right_steering', -0.6),
                ('avoidance_left_duration', 1.0),
                ('avoidance_right_duration', 1.0),
                ('cone_distance_topic', DEFAULT_CONE_DISTANCE_TOPIC_NAME),
                ('cone_avoidance_enabled', True),
                ('cone_trigger_distance_m', 2.0),
                ('cone_reset_distance_m', 2.5)
            ])
        output_topic = self.get_parameter('cmd_vel_output_topic').value
        lane_mode_topic = self.get_parameter('lane_mode_topic').value
        avoidance_cmd_topic = self.get_parameter('avoidance_cmd_topic').value
        cone_distance_topic = self.get_parameter('cone_distance_topic').value
        self.twist_publisher = self.create_publisher(Twist, output_topic, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32, CENTROID_TOPIC_NAME, self.controller, 10)
        self.centroid_subscriber
        self.lane_mode_subscriber = self.create_subscription(String, lane_mode_topic, self.update_lane_mode, 10)
        self.lane_mode_subscriber
        self.avoidance_cmd_subscriber = self.create_subscription(String, avoidance_cmd_topic, self.update_avoidance_cmd, 10)
        self.avoidance_cmd_subscriber
        self.cone_distance_subscriber = self.create_subscription(Float32, cone_distance_topic, self.update_cone_distance, 10)
        self.cone_distance_subscriber

        self.Kp = self.get_parameter('Kp_steering').value # between [0,1]
        self.Ki = self.get_parameter('Ki_steering').value # between [0,1]
        self.Kd = self.get_parameter('Kd_steering').value # between [0,1]
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_throttle = self.get_parameter('zero_throttle').value # between [-1,1] but should be around 0
        self.max_throttle = self.get_parameter('max_throttle').value # between [-1,1]
        self.min_throttle = self.get_parameter('min_throttle').value # between [-1,1]
        self.max_right_steering = self.get_parameter('max_right_steering').value # between [-1,1]
        self.max_left_steering = self.get_parameter('max_left_steering').value # between [-1,1]
        self.left_lane_target_error = self.get_parameter('left_lane_target_error').value
        self.right_lane_target_error = self.get_parameter('right_lane_target_error').value
        self.lane_mode = self.normalize_lane_mode(self.get_parameter('lane_mode').value)
        self.avoidance_throttle = self.get_parameter('avoidance_throttle').value
        self.avoidance_left_steering = self.get_parameter('avoidance_left_steering').value
        self.avoidance_right_steering = self.get_parameter('avoidance_right_steering').value
        self.avoidance_left_duration = self.get_parameter('avoidance_left_duration').value
        self.avoidance_right_duration = self.get_parameter('avoidance_right_duration').value
        self.cone_avoidance_enabled = bool(self.get_parameter('cone_avoidance_enabled').value)
        self.cone_trigger_distance_m = float(self.get_parameter('cone_trigger_distance_m').value)
        self.cone_reset_distance_m = float(self.get_parameter('cone_reset_distance_m').value)

        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8
        self.avoidance_active = False
        self.avoidance_stage = 'idle'
        self.avoidance_stage_start_time = 0.0
        self.cone_trigger_armed = True
        self.avoidance_timer = self.create_timer(self.Ts, self.run_avoidance_step)
        
        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
            f'\ncmd_vel_output_topic: {output_topic}'
            f'\nlane_mode_topic: {lane_mode_topic}'
            f'\nlane_mode: {self.lane_mode}'
            f'\nleft_lane_target_error: {self.left_lane_target_error}'
            f'\nright_lane_target_error: {self.right_lane_target_error}'
            f'\navoidance_cmd_topic: {avoidance_cmd_topic}'
            f'\navoidance_throttle: {self.avoidance_throttle}'
            f'\navoidance_left_steering: {self.avoidance_left_steering}'
            f'\navoidance_right_steering: {self.avoidance_right_steering}'
            f'\navoidance_left_duration: {self.avoidance_left_duration}'
            f'\navoidance_right_duration: {self.avoidance_right_duration}'
            f'\ncone_distance_topic: {cone_distance_topic}'
            f'\ncone_avoidance_enabled: {self.cone_avoidance_enabled}'
            f'\ncone_trigger_distance_m: {self.cone_trigger_distance_m}'
            f'\ncone_reset_distance_m: {self.cone_reset_distance_m}'
        )

    def controller(self, data):
        if self.avoidance_active:
            return

        # setting up PID control
        raw_error = data.data
        target_error = self.get_target_error()
        self.ek = raw_error - target_error

        # Throttle gain scheduling (function of error)
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle)  / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Steering PID terms
        self.proportional_error = self.Kp * self.ek
        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        # Publish values
        try:
            # publish control signals
            self.twist_cmd.angular.z = steering_float
            self.twist_cmd.linear.x = throttle_float
            self.twist_publisher.publish(self.twist_cmd)

            # shift current time and error values to previous values
            self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def update_lane_mode(self, msg):
        requested_mode = self.normalize_lane_mode(msg.data)
        if requested_mode == self.lane_mode:
            return
        self.lane_mode = requested_mode
        self.get_logger().info(
            f'Lane mode changed to {self.lane_mode} '
            f'(target_error={self.get_target_error():.3f})'
        )

    def normalize_lane_mode(self, lane_mode):
        lane_mode = str(lane_mode).strip().lower()
        if lane_mode in ('left', 'right', 'center'):
            return lane_mode
        return 'center'

    def get_target_error(self):
        if self.lane_mode == 'left':
            return self.left_lane_target_error
        if self.lane_mode == 'right':
            return self.right_lane_target_error
        return 0.0

    def update_avoidance_cmd(self, msg):
        requested_cmd = str(msg.data).strip().lower()
        if requested_cmd in ('start', 'avoid', 'go'):
            self.start_avoidance()
        elif requested_cmd in ('cancel', 'stop'):
            self.stop_avoidance()

    def update_cone_distance(self, msg):
        if not self.cone_avoidance_enabled:
            return

        distance_m = float(msg.data)

        if distance_m >= self.cone_reset_distance_m and not self.cone_trigger_armed:
            self.cone_trigger_armed = True
            self.get_logger().info(
                f'Cone avoidance re-armed at {distance_m:.2f} m'
            )
            return

        if self.avoidance_active or not self.cone_trigger_armed:
            return

        if distance_m <= self.cone_trigger_distance_m:
            self.cone_trigger_armed = False
            self.get_logger().info(
                f'Cone detected at {distance_m:.2f} m, triggering avoidance'
            )
            self.start_avoidance()

    def start_avoidance(self):
        self.avoidance_active = True
        self.avoidance_stage = 'left'
        self.avoidance_stage_start_time = self.get_time_seconds()
        self.get_logger().info('Starting scripted avoidance: left then right')

    def stop_avoidance(self):
        if not self.avoidance_active:
            return
        self.avoidance_active = False
        self.avoidance_stage = 'idle'
        self.twist_cmd.linear.x = self.zero_throttle
        self.twist_cmd.angular.z = 0.0
        self.twist_publisher.publish(self.twist_cmd)
        self.get_logger().info('Scripted avoidance canceled')

    def run_avoidance_step(self):
        if not self.avoidance_active:
            return

        current_time = self.get_time_seconds()
        elapsed_time = current_time - self.avoidance_stage_start_time

        if self.avoidance_stage == 'left' and elapsed_time >= self.avoidance_left_duration:
            self.avoidance_stage = 'right'
            self.avoidance_stage_start_time = current_time
            self.get_logger().info('Scripted avoidance: switching from left to right')
        elif self.avoidance_stage == 'right' and elapsed_time >= self.avoidance_right_duration:
            self.avoidance_active = False
            self.avoidance_stage = 'idle'
            self.get_logger().info('Scripted avoidance complete, returning to lane following')
            return

        self.twist_cmd.linear.x = self.avoidance_throttle
        if self.avoidance_stage == 'left':
            self.twist_cmd.angular.z = self.avoidance_left_steering
        else:
            self.twist_cmd.angular.z = self.avoidance_right_steering
        self.twist_publisher.publish(self.twist_cmd)

    def get_time_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()

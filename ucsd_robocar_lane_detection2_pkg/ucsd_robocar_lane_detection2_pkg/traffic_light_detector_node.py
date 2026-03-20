import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

try:
    from inference_sdk import InferenceHTTPClient
except ImportError:
    InferenceHTTPClient = None


NODE_NAME = 'traffic_light_detector_node'
DEFAULT_CAMERA_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_raw'
DEFAULT_STATE_TOPIC = '/traffic_light/state'
DEFAULT_DISTANCE_TOPIC = '/traffic_light/distance_m'


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', DEFAULT_CAMERA_TOPIC),
                ('depth_topic', DEFAULT_DEPTH_TOPIC),
                ('traffic_light_state_topic', DEFAULT_STATE_TOPIC),
                ('traffic_light_distance_topic', DEFAULT_DISTANCE_TOPIC),
                ('inference_server_url', 'http://localhost:9001'),
                ('roboflow_model_id', 'traffic-light-training/6'),
                ('api_key', 'UOi4p4RmZsJV6kqNi8ln'),
                ('image_path', '/tmp/traffic_light_detector.jpg'),
                ('debug_capture_dir', '/tmp/traffic_light_debug'),
                ('frame_skip', 1),
                ('confidence_threshold', 0.5),
                ('log_interval_sec', 1.0),
                ('state_hold_sec', 2.0),
                ('max_depth_age_sec', 0.2),
                ('depth_roi_scale', 2.5),
                ('min_depth_roi_size_px', 18),
                ('depth_percentile', 20.0),
                ('max_distance_m', 10.0),
            ],
        )

        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        state_topic = self.get_parameter('traffic_light_state_topic').value
        distance_topic = self.get_parameter('traffic_light_distance_topic').value
        self.inference_server_url = self.get_parameter('inference_server_url').value
        self.model_id = self.get_parameter('roboflow_model_id').value
        self.api_key = self.get_parameter('api_key').value
        self.image_path = self.get_parameter('image_path').value
        self.debug_capture_dir = self.get_parameter('debug_capture_dir').value
        self.frame_skip = max(1, int(self.get_parameter('frame_skip').value))
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.state_hold_sec = float(self.get_parameter('state_hold_sec').value)
        self.max_depth_age_sec = float(self.get_parameter('max_depth_age_sec').value)
        self.depth_roi_scale = float(self.get_parameter('depth_roi_scale').value)
        self.min_depth_roi_size_px = int(self.get_parameter('min_depth_roi_size_px').value)
        self.depth_percentile = float(self.get_parameter('depth_percentile').value)
        self.max_distance_m = float(self.get_parameter('max_distance_m').value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_published_state = 'unknown'
        self.last_logged_key = 'startup'
        self.last_log_time = 0.0
        self.last_valid_state = None
        self.last_valid_confidence = 0.0
        self.last_valid_detection_time = 0.0
        self.last_valid_distance_m = None
        self.last_distance_unavailable_reason = None
        self.latest_depth_frame = None
        self.latest_depth_time = 0.0
        self.latest_rgb_frame = None
        self.latest_frame_id = 0
        self.last_processed_frame_id = 0
        self.inference_in_progress = False
        self.data_lock = threading.Lock()
        self.client = None
        os.makedirs(self.debug_capture_dir, exist_ok=True)

        if InferenceHTTPClient is None:
            self.get_logger().error(
                'inference_sdk is not installed in this ROS2 environment. '
                'Traffic-light detection will stay inactive until it is installed.'
            )
        else:
            self.client = InferenceHTTPClient(
                api_url=self.inference_server_url,
                api_key=self.api_key,
            )

        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.distance_publisher = self.create_publisher(Float32, distance_topic, 10)
        self.camera_subscriber = self.create_subscription(
            Image, camera_topic, self.update_latest_image, 1
        )
        self.depth_subscriber = self.create_subscription(
            Image, depth_topic, self.update_depth_frame, 1
        )
        self.processing_timer = self.create_timer(0.02, self.maybe_process_latest_frame)
        self.camera_subscriber
        self.depth_subscriber
        self.processing_timer

        self.get_logger().info(
            f'camera_topic: {camera_topic}\n'
            f'depth_topic: {depth_topic}\n'
            f'traffic_light_state_topic: {state_topic}\n'
            f'traffic_light_distance_topic: {distance_topic}\n'
            f'inference_server_url: {self.inference_server_url}\n'
            f'roboflow_model_id: {self.model_id}\n'
            f'frame_skip: {self.frame_skip}\n'
            f'confidence_threshold: {self.confidence_threshold}\n'
            f'log_interval_sec: {self.log_interval_sec}\n'
            f'state_hold_sec: {self.state_hold_sec}\n'
            f'max_depth_age_sec: {self.max_depth_age_sec}\n'
            f'depth_roi_scale: {self.depth_roi_scale}\n'
            f'min_depth_roi_size_px: {self.min_depth_roi_size_px}\n'
            f'depth_percentile: {self.depth_percentile}\n'
            f'max_distance_m: {self.max_distance_m}'
        )

    def update_latest_image(self, image_msg):
        if self.client is None:
            return

        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        with self.data_lock:
            self.latest_rgb_frame = frame
            self.latest_frame_id += 1

    def maybe_process_latest_frame(self):
        if self.client is None or self.inference_in_progress:
            return

        with self.data_lock:
            if self.latest_rgb_frame is None:
                return
            if self.latest_frame_id == self.last_processed_frame_id:
                return

            frame = self.latest_rgb_frame.copy()
            frame_id = self.latest_frame_id
            depth_frame_snapshot = None
            depth_age_snapshot = None
            if self.latest_depth_frame is not None:
                depth_frame_snapshot = self.latest_depth_frame.copy()
                depth_age_snapshot = time.time() - self.latest_depth_time

        self.inference_in_progress = True
        inference_thread = threading.Thread(
            target=self.run_inference,
            args=(frame, frame_id, depth_frame_snapshot, depth_age_snapshot),
            daemon=True,
        )
        inference_thread.start()

    def run_inference(self, frame, frame_id, depth_frame_snapshot, depth_age_snapshot):
        try:
            cv2.imwrite(self.image_path, frame)
            result = self.client.infer(self.image_path, model_id=self.model_id)
            self.handle_inference_result(frame, result, depth_frame_snapshot, depth_age_snapshot)

            with self.data_lock:
                if frame_id > self.last_processed_frame_id:
                    self.last_processed_frame_id = frame_id
        except Exception as exc:
            self.get_logger().error(f'Inference error: {exc}')
        finally:
            self.inference_in_progress = False

    def handle_inference_result(self, frame, result, depth_frame_snapshot, depth_age_snapshot):
        predictions = result.get('predictions', [])
        best_prediction = self.get_best_valid_prediction(predictions)
        if best_prediction is None:
            held_state = self.get_held_state()
            if held_state is not None:
                self.publish_state(held_state, self.last_valid_confidence)
                self.publish_distance(self.last_valid_distance_m)
                self.log_detection_status(
                    'hold',
                    held_state,
                    self.last_valid_confidence,
                    self.last_valid_distance_m,
                    frame=frame,
                )
            elif self.should_log_no_detection(predictions):
                self.log_detection_status(
                    'none',
                    raw_prediction_summary=self.summarize_predictions(predictions),
                    frame=frame,
                )
            return

        confidence = float(best_prediction.get('confidence', 0.0))
        detected_state = self.normalize_label(best_prediction.get('class', 'unknown'))
        distance_m, distance_unavailable_reason = self.estimate_distance(
            best_prediction,
            depth_frame_snapshot,
            depth_age_snapshot,
        )
        if (
            distance_m is None
            and self.last_valid_state == detected_state
            and self.last_valid_distance_m is not None
            and time.time() - self.last_valid_detection_time <= self.state_hold_sec
        ):
            distance_m = self.last_valid_distance_m
            distance_unavailable_reason = None
        self.last_valid_state = detected_state
        self.last_valid_confidence = confidence
        self.last_valid_detection_time = time.time()
        if distance_m is not None:
            self.last_valid_distance_m = distance_m
            self.last_distance_unavailable_reason = None
        else:
            self.last_distance_unavailable_reason = distance_unavailable_reason
        self.publish_state(detected_state, confidence)
        self.publish_distance(distance_m)
        self.log_detection_status(
            'detected',
            detected_state,
            confidence,
            distance_m,
            distance_unavailable_reason,
            frame=frame,
        )

    def update_depth_frame(self, depth_msg):
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        with self.data_lock:
            self.latest_depth_frame = depth_frame
            self.latest_depth_time = time.time()

    def get_best_valid_prediction(self, predictions):
        valid_predictions = []
        for prediction in predictions:
            detected_state = self.normalize_label(prediction.get('class', ''))
            confidence = float(prediction.get('confidence', 0.0))
            if detected_state == 'unknown' or confidence < self.confidence_threshold:
                continue
            valid_predictions.append(prediction)

        if not valid_predictions:
            return None

        return max(valid_predictions, key=lambda prediction: prediction.get('confidence', 0.0))

    def normalize_label(self, label):
        label = str(label).strip().lower()
        if label in ('red', 'red light'):
            return 'red'
        if label in ('green', 'green light'):
            return 'green'
        if label in ('yellow', 'yellow light'):
            return 'yellow'
        return 'unknown'

    def publish_state(self, state, confidence):
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        if state != self.last_published_state:
            self.last_published_state = state

    def publish_distance(self, distance_m):
        if distance_m is None:
            return
        msg = Float32()
        msg.data = float(distance_m)
        self.distance_publisher.publish(msg)

    def get_held_state(self):
        if self.last_valid_state is None:
            return None
        if time.time() - self.last_valid_detection_time > self.state_hold_sec:
            return None
        return self.last_valid_state

    def should_log_no_detection(self, predictions):
        if self.last_valid_state is None:
            return False
        return True

    def summarize_predictions(self, predictions):
        if not predictions:
            return 'raw_predictions=0'

        top_prediction = max(predictions, key=lambda prediction: float(prediction.get('confidence', 0.0)))
        top_class = str(top_prediction.get('class', 'unknown')).strip().lower() or 'unknown'
        top_confidence = float(top_prediction.get('confidence', 0.0))
        normalized_top_class = self.normalize_label(top_class)
        summary = (
            f'raw_predictions={len(predictions)}, '
            f'top_prediction={top_class}:{top_confidence:.2f}, '
            f'normalized_top_class={normalized_top_class}, '
            f'confidence_threshold={self.confidence_threshold:.2f}'
        )
        if normalized_top_class == 'unknown':
            summary += ', filtered_because=unknown_label'
        elif top_confidence < self.confidence_threshold:
            summary += ', filtered_because=below_confidence_threshold'
        else:
            summary += ', filtered_because=not_selected'
        return summary

    def estimate_distance(self, prediction, depth_frame=None, depth_age_sec=None):
        if depth_frame is None:
            depth_frame = self.latest_depth_frame
            if self.latest_depth_frame is not None:
                depth_age_sec = time.time() - self.latest_depth_time

        if depth_frame is None:
            return None, 'no_depth_frame_yet'
        if depth_age_sec is None:
            return None, 'unknown_depth_age'
        if depth_age_sec > self.max_depth_age_sec:
            return None, 'stale_depth_frame'

        center_x = int(round(float(prediction.get('x', 0.0))))
        center_y = int(round(float(prediction.get('y', 0.0))))
        box_width = max(
            self.min_depth_roi_size_px,
            int(round(float(prediction.get('width', 0.0)) * self.depth_roi_scale)),
        )
        box_height = max(
            self.min_depth_roi_size_px,
            int(round(float(prediction.get('height', 0.0)) * self.depth_roi_scale)),
        )

        x_min = max(0, center_x - box_width // 2)
        x_max = min(depth_frame.shape[1], center_x + box_width // 2)
        y_min = max(0, center_y - box_height // 2)
        y_max = min(depth_frame.shape[0], center_y + box_height // 2)

        roi = depth_frame[y_min:y_max, x_min:x_max]
        if roi.size == 0:
            return None, 'empty_depth_roi'

        valid_depths = roi[(roi > 0) & np.isfinite(roi)]
        if valid_depths.size == 0:
            return None, 'no_valid_depth_pixels'

        distance_m = float(np.percentile(valid_depths, self.depth_percentile)) / 1000.0
        if distance_m > self.max_distance_m:
            return None, 'depth_out_of_range'

        return distance_m, None

    def log_detection_status(
        self,
        status,
        state=None,
        confidence=None,
        distance_m=None,
        distance_unavailable_reason=None,
        raw_prediction_summary=None,
        frame=None,
    ):
        current_time = time.time()
        log_key = status if state is None else f'{status}:{state}'
        should_log = (
            log_key != self.last_logged_key
            or current_time - self.last_log_time >= self.log_interval_sec
        )
        if not should_log:
            return

        self.save_debug_frame(
            frame,
            status,
            state=state,
            confidence=confidence,
            raw_prediction_summary=raw_prediction_summary,
        )

        if status == 'none':
            message = 'Perception status: no traffic light detected'
            if raw_prediction_summary is not None:
                message += f' ({raw_prediction_summary})'
            self.get_logger().info(message)
        elif status == 'hold':
            message = f'Perception status: holding traffic_light={state} (last_confidence={confidence:.2f}'
            if distance_m is not None:
                message += f', last_distance={distance_m:.2f} m'
            elif self.last_distance_unavailable_reason is not None:
                message += (
                    f', distance_unavailable_because={self.last_distance_unavailable_reason}'
                )
            message += ')'
            self.get_logger().info(message)
        else:
            message = f'Perception status: traffic_light={state} (confidence={confidence:.2f}'
            if distance_m is not None:
                message += f', distance={distance_m:.2f} m'
            elif distance_unavailable_reason is not None:
                message += f', distance_unavailable_because={distance_unavailable_reason}'
            message += ')'
            self.get_logger().info(message)

        self.last_logged_key = log_key
        self.last_log_time = current_time

    def save_debug_frame(
        self,
        frame,
        status,
        state=None,
        confidence=None,
        raw_prediction_summary=None,
    ):
        if frame is None:
            return

        timestamp = time.strftime('%Y%m%d-%H%M%S')
        millis = int((time.time() % 1) * 1000)
        label = status
        if state is not None:
            label = f'{status}_{state}'
        if confidence is not None:
            label += f'_{confidence:.2f}'
        if raw_prediction_summary is not None and 'raw_predictions=0' in raw_prediction_summary:
            label += '_raw0'
        filename = f'{timestamp}-{millis:03d}_{label}.jpg'
        output_path = os.path.join(self.debug_capture_dir, filename)
        try:
            cv2.imwrite(output_path, frame)
        except Exception as exc:
            self.get_logger().warning(f'Failed to save debug frame to {output_path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    detector = TrafficLightDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

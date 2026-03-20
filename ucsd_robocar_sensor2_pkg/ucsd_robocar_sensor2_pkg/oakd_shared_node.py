import time
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

try:
    import depthai as dai
except ImportError:
    dai = None

try:
    from inference_sdk import InferenceHTTPClient
except ImportError:
    InferenceHTTPClient = None


NODE_NAME = 'oakd_shared_node'
DEFAULT_CAMERA_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_raw'
DEFAULT_CONE_DISTANCE_TOPIC = '/cone_distance_m'


class OakDSharedNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', DEFAULT_CAMERA_TOPIC),
                ('depth_topic', DEFAULT_DEPTH_TOPIC),
                ('cone_distance_topic', DEFAULT_CONE_DISTANCE_TOPIC),
                ('inference_server_url', 'http://localhost:9001'),
                ('roboflow_model_id', 'traffic-light-training/6'),
                ('api_key', 'UOi4p4RmZsJV6kqNi8ln'),
                ('image_path', '/tmp/oakd_shared_cone_detector.jpg'),
                ('frame_skip', 2),
                ('confidence_threshold', 0.5),
                ('target_classes_csv', 'orange cone,cone'),
                ('log_interval_sec', 0.5),
                ('reconnect_interval_sec', 2.0),
                ('camera_fps', 15.0),
                ('rgb_width', 640),
                ('rgb_height', 400),
            ],
        )

        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.cone_distance_topic = self.get_parameter('cone_distance_topic').value
        self.inference_server_url = self.get_parameter('inference_server_url').value
        self.model_id = self.get_parameter('roboflow_model_id').value
        self.api_key = self.get_parameter('api_key').value
        self.image_path = self.get_parameter('image_path').value
        self.frame_skip = max(1, int(self.get_parameter('frame_skip').value))
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.target_classes = {
            self.normalize_label(item)
            for item in str(self.get_parameter('target_classes_csv').value).split(',')
            if self.normalize_label(item)
        }
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.reconnect_interval_sec = float(self.get_parameter('reconnect_interval_sec').value)
        self.camera_fps = float(self.get_parameter('camera_fps').value)
        self.rgb_width = int(self.get_parameter('rgb_width').value)
        self.rgb_height = int(self.get_parameter('rgb_height').value)

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, self.camera_topic, 10)
        self.depth_publisher = self.create_publisher(Image, self.depth_topic, 10)
        self.distance_publisher = self.create_publisher(Float32, self.cone_distance_topic, 10)

        self.client = None
        self.device = None
        self.rgb_queue = None
        self.depth_queue = None
        self.frame_count = 0
        self.last_log_time = 0.0
        self.last_device_error_log_time = 0.0
        self.last_reconnect_attempt_time = 0.0
        self.inference_lock = threading.Lock()
        self.inference_in_progress = False
        self.latest_rgb_frame = None
        self.latest_depth_frame = None

        if dai is None:
            self.get_logger().error(
                'depthai is not installed in this ROS2 environment. '
                'OAK-D shared node cannot start.'
            )
            return

        if InferenceHTTPClient is None:
            self.get_logger().warning(
                'inference_sdk is not installed in this ROS2 environment. '
                'RGB publishing will continue, but cone-distance detection is disabled.'
            )
        else:
            self.client = InferenceHTTPClient(
                api_url=self.inference_server_url,
                api_key=self.api_key,
            )

        self.initialize_device()
        self.capture_timer = self.create_timer(0.02, self.process_frame)

        self.get_logger().info(
            f'camera_topic: {self.camera_topic}\n'
            f'depth_topic: {self.depth_topic}\n'
            f'cone_distance_topic: {self.cone_distance_topic}\n'
            f'inference_server_url: {self.inference_server_url}\n'
            f'roboflow_model_id: {self.model_id}\n'
            f'frame_skip: {self.frame_skip}\n'
            f'confidence_threshold: {self.confidence_threshold}\n'
            f'target_classes: {sorted(self.target_classes)}\n'
            f'reconnect_interval_sec: {self.reconnect_interval_sec}\n'
            f'camera_fps: {self.camera_fps}\n'
            f'rgb_size: ({self.rgb_width}, {self.rgb_height})'
        )

    def setup_oakd_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.rgb_width, self.rgb_height)
        cam_rgb.setFps(self.camera_fps)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(self.camera_fps)
        mono_right.setFps(self.camera_fps)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(self.rgb_width, self.rgb_height)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        self.device = dai.Device(pipeline)
        self.rgb_queue = self.device.getOutputQueue(name='rgb', maxSize=1, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name='depth', maxSize=1, blocking=False)

    def initialize_device(self):
        self.cleanup_device()
        try:
            self.setup_oakd_pipeline()
            self.get_logger().info('Connected to OAK-D Lite shared RGB/depth pipeline.')
        except Exception as exc:
            self.handle_device_error(exc, 'Failed to initialize OAK-D Lite stream')

    def cleanup_device(self):
        self.rgb_queue = None
        self.depth_queue = None
        self.latest_rgb_frame = None
        self.latest_depth_frame = None
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None

    def handle_device_error(self, exc, prefix='OAK-D communication error'):
        current_time = time.time()
        if current_time - self.last_device_error_log_time >= self.log_interval_sec:
            self.get_logger().warning(
                f'{prefix}: {exc}. '
                f'Will retry in {self.reconnect_interval_sec:.1f} seconds.'
            )
            self.last_device_error_log_time = current_time
        self.cleanup_device()
        self.last_reconnect_attempt_time = current_time

    def maybe_reconnect_device(self):
        current_time = time.time()
        if current_time - self.last_reconnect_attempt_time < self.reconnect_interval_sec:
            return
        self.last_reconnect_attempt_time = current_time
        self.initialize_device()

    def get_latest_packet(self, queue):
        latest_packet = None
        while True:
            packet = queue.tryGet()
            if packet is None:
                return latest_packet
            latest_packet = packet

    def process_frame(self):
        if self.device is None or self.rgb_queue is None or self.depth_queue is None:
            self.maybe_reconnect_device()
            return

        try:
            rgb_packet = self.get_latest_packet(self.rgb_queue)
            depth_packet = self.get_latest_packet(self.depth_queue)
        except RuntimeError as exc:
            self.handle_device_error(exc)
            return

        if rgb_packet is None or depth_packet is None:
            return

        rgb_frame = rgb_packet.getCvFrame()
        depth_frame = depth_packet.getFrame()
        self.latest_rgb_frame = rgb_frame.copy()
        self.latest_depth_frame = depth_frame.copy()
        self.publish_depth_frame(depth_frame)
        self.publish_rgb_frame(rgb_frame)

        if self.client is None:
            return

        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        if self.inference_in_progress:
            return

        self.inference_in_progress = True
        inference_thread = threading.Thread(
            target=self.run_cone_inference,
            args=(self.latest_rgb_frame.copy(), self.latest_depth_frame.copy()),
            daemon=True,
        )
        inference_thread.start()

    def publish_rgb_frame(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)

    def publish_depth_frame(self, depth_frame):
        msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
        self.depth_publisher.publish(msg)

    def run_cone_inference(self, rgb_frame, depth_frame):
        try:
            with self.inference_lock:
                cv2.imwrite(self.image_path, rgb_frame)
                result = self.client.infer(self.image_path, model_id=self.model_id)

            best_prediction = self.get_best_cone_prediction(result.get('predictions', []))
            if best_prediction is None:
                return

            distance_m = self.estimate_distance(depth_frame, best_prediction)
            if distance_m is None:
                return

            distance_msg = Float32()
            distance_msg.data = distance_m
            self.distance_publisher.publish(distance_msg)

            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval_sec:
                cone_label = best_prediction.get('class', 'cone')
                confidence = float(best_prediction.get('confidence', 0.0))
                self.get_logger().info(
                    f'Perception status: cone={cone_label} '
                    f'(distance={distance_m:.2f} m, confidence={confidence:.2f})'
                )
                self.last_log_time = current_time
        except Exception as exc:
            self.get_logger().error(f'Inference error: {exc}')
        finally:
            self.inference_in_progress = False

    def get_best_cone_prediction(self, predictions):
        cone_predictions = []
        for prediction in predictions:
            label = self.normalize_label(prediction.get('class', ''))
            confidence = float(prediction.get('confidence', 0.0))
            if label in self.target_classes and confidence >= self.confidence_threshold:
                cone_predictions.append(prediction)
        if not cone_predictions:
            return None
        return max(cone_predictions, key=lambda prediction: prediction.get('confidence', 0.0))

    def normalize_label(self, label):
        return str(label).strip().lower().replace('_', ' ').replace('-', ' ')

    def estimate_distance(self, depth_frame, prediction):
        center_x = int(round(float(prediction.get('x', 0.0))))
        center_y = int(round(float(prediction.get('y', 0.0))))
        box_width = max(6, int(round(float(prediction.get('width', 0.0)) * 0.4)))
        box_height = max(6, int(round(float(prediction.get('height', 0.0)) * 0.4)))

        x_min = max(0, center_x - box_width // 2)
        x_max = min(depth_frame.shape[1], center_x + box_width // 2)
        y_min = max(0, center_y - box_height // 2)
        y_max = min(depth_frame.shape[0], center_y + box_height // 2)

        roi = depth_frame[y_min:y_max, x_min:x_max]
        if roi.size == 0:
            return None

        valid_depths = roi[(roi > 0) & np.isfinite(roi)]
        if valid_depths.size == 0:
            return None

        return float(np.median(valid_depths)) / 1000.0


def main(args=None):
    rclpy.init(args=args)
    node = OakDSharedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        node.cleanup_device()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

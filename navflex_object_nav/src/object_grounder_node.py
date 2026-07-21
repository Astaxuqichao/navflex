#!/usr/bin/env python3
"""Object grounder — the keystone of object-goal navigation.

Subscribes to the robot's live ego-view camera and answers, on demand, a single
question: "is {target} visible right now, and roughly where?" It turns the VLM's
image-space answer into a BEARING in base_link (+left), which the approach phase
projects into the LIDAR to get a range. This is object grounding: language -> a
direction in the sensor world.

Same camera as exploration (MATRiX xgb ego cam): /image_raw/compressed, 90 deg
FOV, forward-mounted on base_link. Same VLM backends as the world-model critic.

Service:  navflex_object_nav/detect   (navflex_object_nav/srv/DetectObject)
Events:   navflex_object_nav/grounder_events  (std_msgs/String, JSON)
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from navflex_obj.vlm_detect import DetectorError, build_detector
from navflex_object_nav.srv import DetectObject


class ObjectGrounder(Node):
    def __init__(self) -> None:
        super().__init__('object_grounder')

        # Camera — identical to the exploration / world-model setup.
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('image_max_age', 2.0)
        self.declare_parameter('camera_fov_deg', 90.0)
        # +left (CCW) means x_frac<0.5 (left of image) -> positive bearing. Flip
        # if the sim mirrors the image horizontally.
        self.declare_parameter('bearing_left_positive', True)

        # VLM backend — mirrors navflex_wm.critic params so one key/aggregator
        # serves both the detector and the safety critic.
        self.declare_parameter('detector', 'null')          # openai_compat | claude | null
        self.declare_parameter('detector_model', '')
        self.declare_parameter('detector_base_url', '')     # openai_compat root, .../v1
        self.declare_parameter('detector_api_key_env', '')  # env var holding the key
        self.declare_parameter('detector_proxy_env', 'HTTPS_PROXY')
        self.declare_parameter('detector_timeout', 60.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.image_max_age = float(self.get_parameter('image_max_age').value)
        self.camera_fov_rad = math.radians(float(self.get_parameter('camera_fov_deg').value))
        self.bearing_left_positive = bool(self.get_parameter('bearing_left_positive').value)

        self.detector = self._build_detector()

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            CompressedImage, self.image_topic, self._on_image, image_qos)

        self.event_pub = self.create_publisher(
            String, 'navflex_object_nav/grounder_events', 10)
        self.create_service(DetectObject, 'navflex_object_nav/detect', self._on_detect)

        self._image_lock = threading.Lock()
        self._latest_image = None
        self._latest_stamp = 0.0
        # A VLM request can outlive the middleware service-query lifetime. Keep
        # slow HTTP work out of the ROS callback: the first request starts a
        # worker and returns ``detection pending``; short polls collect it while
        # the robot holds the current viewing direction.
        self._job_lock = threading.Lock()
        self._job = None

        health = self.detector.health()
        if not health.startswith('ok'):
            self.get_logger().error(
                f"detector '{self.detector.name}' not usable: {health}. "
                f"Every detect call will report ok=false / visible=false.")
        self.get_logger().info(
            f"Object grounder ready. detector='{self.detector.name}' ({health}) "
            f"image_topic='{self.image_topic}' fov={math.degrees(self.camera_fov_rad):.0f}deg")

    DEFAULT_KEY_ENV = {
        'claude': 'ANTHROPIC_API_KEY',
        'openai_compat': 'NAVFLEX_CRITIC_API_KEY',
    }

    def _build_detector(self):
        import os
        kind = (self.get_parameter('detector').value or 'null').strip().lower()
        key_env = (self.get_parameter('detector_api_key_env').value
                   or self.DEFAULT_KEY_ENV.get(kind, ''))
        proxy_env = self.get_parameter('detector_proxy_env').value
        proxy = (os.environ.get(proxy_env) or os.environ.get(proxy_env.lower(), '')
                 ) if proxy_env else ''
        try:
            return build_detector(
                kind,
                base_url=self.get_parameter('detector_base_url').value,
                model=self.get_parameter('detector_model').value or None,
                api_key=os.environ.get(key_env, '') if key_env else '',
                proxy=proxy,
                timeout=float(self.get_parameter('detector_timeout').value))
        except DetectorError as exc:
            self.get_logger().warn(
                f"detector '{kind}' unavailable ({exc}); using null detector")
            return build_detector('null')

    def _on_image(self, msg: CompressedImage) -> None:
        with self._image_lock:
            self._latest_image = bytes(msg.data)
            self._latest_stamp = time.monotonic()

    def _take_image(self):
        with self._image_lock:
            if self._latest_image is None:
                return None
            age = time.monotonic() - self._latest_stamp
            if age > self.image_max_age:
                self.get_logger().warn(
                    f'latest frame is {age:.1f}s old (max {self.image_max_age:.1f}s)')
                return None
            return self._latest_image

    def _on_detect(self, request, response):
        target = (request.target or '').strip()
        if not target:
            response.ok = False
            response.notes = 'empty target'
            return response

        with self._job_lock:
            job = self._job
            if job is not None and job['target'] != target:
                response.ok = False
                response.notes = 'detector busy with another target'
                return response
            if job is None:
                image = self._take_image()
                if image is None:
                    response.ok = False
                    response.notes = f'no fresh frame on {self.image_topic}'
                    self._publish_event(target, response, extra='no_image')
                    return response
                job = {'target': target, 'done': False, 'det': None, 'error': None}
                self._job = job
                threading.Thread(
                    target=self._run_detection,
                    args=(job, image, target),
                    daemon=True).start()
            if not job['done']:
                response.ok = False
                response.notes = 'detection pending'
                return response
            self._job = None
            det = job['det']
            error = job['error']

        if error is not None:
            response.ok = False
            response.notes = f'detector error: {error}'
            self._publish_event(target, response, extra='error')
            return response

        response.ok = True
        response.visible = det.visible
        response.confidence = det.confidence
        response.x_frac = det.x_frac
        response.y_frac = det.y_frac
        # image x_frac (0=left..1=right) -> bearing. +left convention by default.
        offset = (0.5 - det.x_frac)  # >0 when target is left-of-centre
        if not self.bearing_left_positive:
            offset = -offset
        response.bearing_rad = offset * self.camera_fov_rad
        response.bearing_deg = math.degrees(response.bearing_rad)
        response.notes = det.reason
        response.raw_json = json.dumps(det.raw, ensure_ascii=False)

        self.get_logger().info(
            f"detect('{target}'): visible={det.visible} conf={det.confidence:.2f} "
            f"x_frac={det.x_frac:.2f} bearing={response.bearing_deg:+.0f}deg | {det.reason}")
        self._publish_event(target, response)
        return response

    def _run_detection(self, job, image: bytes, target: str) -> None:
        det = None
        error = None
        try:
            det = self.detector.detect(image, target)
        except DetectorError as exc:
            error = exc
            self.get_logger().warn(f"detect('{target}') failed: {exc}")
        with self._job_lock:
            job['det'] = det
            job['error'] = error
            job['done'] = True

    def _publish_event(self, target, response, extra=''):
        payload = {
            'stamp': time.time(), 'target': target, 'ok': bool(response.ok),
            'visible': bool(getattr(response, 'visible', False)),
            'confidence': float(getattr(response, 'confidence', 0.0)),
            'bearing_deg': float(getattr(response, 'bearing_deg', 0.0)),
            'notes': response.notes,
        }
        if extra:
            payload['status'] = extra
        self.event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectGrounder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

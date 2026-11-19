# Copyright 2026 Claire Liu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Cone Detector node: find traffic cones in camera images with YOLO and name their colors.

Subscribes to the camera image and publishes, for every image, a vision_msgs/
Detection2DArray on cone_detections with the image's header. In each detection,
results[0].hypothesis.class_id is the cone's color ('unknown' if its pixels show no clear
color) and score the detector's confidence; the box center is in pixels with pixel (i, j)
centered at (i, j), the convention of camera_info. With publish_image, the image with the
cones boxed is published on cone_detections/image (view it with rqt_image_view).
"""

from cs5335_cone_finder.color import ColorSettings, DEFAULT_HUE_RANGES
from cs5335_cone_finder.parameters import declare
from cs5335_cone_finder.vision import draw_cones
from cs5335_cone_finder.yolo_detector import YoloConeDetector
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

UNKNOWN_COLOR = 'unknown'


class ConeDetector(Node):
    """
    Cone Detector node.

    Subscribes to the camera image (image_topic); publishes cone_detections
    (vision_msgs/Detection2DArray) and cone_detections/image (sensor_msgs/Image).
    """

    def __init__(self):
        super().__init__('cone_detector')
        model = declare(self, 'model', 'yolov8s-worldv2.pt',
                        'YOLO model file, or a model name ultralytics downloads on first use')
        classes = declare(self, 'classes', ['traffic cone'],
                          'Class names that count as cones (text prompts for YOLO-World)')
        confidence = declare(self, 'confidence', 0.2,
                             'Minimum detector confidence for a cone (0-1)', 0.0, 1.0)
        image_size = declare(self, 'image_size', 320,
                             'Size (px) the detector scales each image to', 32, 1280)
        device = declare(self, 'device', '',
                         "Where the model runs: 'cpu', '0' for the first GPU, '' = automatic")
        image_topic = declare(self, 'image_topic', 'oakd/rgb/preview/image_raw',
                              'Camera image topic')
        self._publish_image = declare(self, 'publish_image', True,
                                      'Also publish the image with the cones boxed')
        colors = declare(self, 'colors', list(DEFAULT_HUE_RANGES),
                         f'Colors a cone may have, from {list(DEFAULT_HUE_RANGES)}')
        hue_ranges = {}
        for name, (low, high) in DEFAULT_HUE_RANGES.items():
            hue_ranges[name] = declare(
                self, f'hue_ranges.{name}', [low, high],
                f'Hue range [low, high] of {name} on the OpenCV scale (0-179, degrees / 2); '
                'low > high wraps through 0')
        settings = ColorSettings(
            hue_ranges=self._check_colors(colors, hue_ranges),
            min_saturation=declare(self, 'min_saturation', 80,
                                   'Minimum saturation (0-255) of a pixel that counts '
                                   'toward a color', 0, 255),
            min_value=declare(self, 'min_value', 60,
                              'Minimum value, or brightness, (0-255) of a pixel that counts '
                              'toward a color', 0, 255),
            min_fraction=declare(self, 'min_color_fraction', 0.15,
                                 'Fraction (0-1) of the pixels looked at that must have a '
                                 'color to name it', 0.0, 1.0),
            region=declare(self, 'color_region', 0.6,
                           'Fraction (0-1) of the box width and height, centered, whose '
                           'pixels decide the color', 0.1, 1.0))

        self._bridge = CvBridge()
        self.get_logger().info(f'Loading the YOLO model {model} for {classes}...')
        self._detector = YoloConeDetector(model, classes, confidence, image_size, device,
                                          settings)
        self._detections_pub = self.create_publisher(Detection2DArray, 'cone_detections', 10)
        self._image_pub = self.create_publisher(Image, 'cone_detections/image', 1)
        # keep only the newest image: while YOLO works on one, older ones are not wanted
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self._image_callback, image_qos)
        self.get_logger().info(f'Model loaded; looking for cones in {image_topic}')

    @staticmethod
    def _check_colors(colors, hue_ranges):
        """Return the hue ranges of the chosen colors; raise ValueError if one is invalid."""
        unknown = [c for c in colors if c not in hue_ranges]
        if unknown:
            raise ValueError(f'Unknown color(s) {unknown}; choose from {list(hue_ranges)}')
        chosen = {}
        for name in colors:
            values = list(hue_ranges[name])
            if len(values) != 2 or not all(0 <= h <= 179 for h in values):
                raise ValueError(f'hue_ranges.{name} needs two hues from 0 to 179')
            chosen[name] = (values[0], values[1])
        return chosen

    def _image_callback(self, msg):
        """
        Find the cones in an image and publish them.

        This is the node's only job, so YOLO runs right here; images that arrive meanwhile
        are dropped (queue depth 1), and the next call gets the newest one.
        """
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Cannot convert the image: {e}', throttle_duration_sec=5.0)
            return
        try:
            cones = self._detector.detect(bgr)
        except Exception as e:  # keep running if one image fails; say why
            self.get_logger().error(f'YOLO failed on an image: {e}', throttle_duration_sec=5.0)
            return
        out = Detection2DArray()
        out.header = msg.header
        for cone in cones:
            detection = Detection2D()
            detection.header = msg.header
            # ultralytics puts pixel edges at whole numbers; camera_info, pixel centers
            detection.bbox.center.position.x = (cone.x1 + cone.x2) / 2.0 - 0.5
            detection.bbox.center.position.y = (cone.y1 + cone.y2) / 2.0 - 0.5
            detection.bbox.size_x = cone.x2 - cone.x1
            detection.bbox.size_y = cone.y2 - cone.y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = cone.color or UNKNOWN_COLOR
            hypothesis.hypothesis.score = cone.confidence
            detection.results.append(hypothesis)
            out.detections.append(detection)
        self._detections_pub.publish(out)
        if self._publish_image and self._image_pub.get_subscription_count() > 0:
            image = self._bridge.cv2_to_imgmsg(draw_cones(bgr, cones), encoding='bgr8')
            image.header = msg.header
            self._image_pub.publish(image)


def main(args=None):
    """Run the Cone Detector node until Ctrl-C."""
    rclpy.init(args=args)
    try:
        node = ConeDetector()
    except Exception as e:  # bad parameters, no ultralytics, model not found or not loaded
        get_logger('cone_detector').fatal(f'Cannot start: {e}')
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

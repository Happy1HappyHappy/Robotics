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
Cone Detector node: find traffic cones of the target colors in the camera image.

For every image it publishes the accepted cones as a vision_msgs/Detection2DArray on
cone_detections, with the image's header. In each detection, results[0].hypothesis.class_id
is the color and score the confidence (a heuristic, not a probability); bbox.center is the
cone's centroid and bbox.size its bounding box, in pixels with pixel (i, j) centered at
(i, j) as in camera_info. Rejected candidates are not published; the debug image on
cone_detections/debug shows them labeled UNKNOWN, and cone_detections/mask shows the
target colors' masks.
"""

from cs5335_cone_navigation.cone_detection import (DEFAULT_HSV, DEFAULT_WEIGHTS,
                                                   detect_cones, DetectionSettings,
                                                   draw_candidates, parse_ranges,
                                                   target_mask)
from cs5335_cone_navigation.parameters import declare
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ConeDetector(Node):
    """
    Cone Detector node.

    Subscribes to the camera image (image_topic); publishes cone_detections,
    cone_detections/debug and cone_detections/mask.
    """

    def __init__(self):
        super().__init__('cone_detector')
        image_topic = declare(self, 'image_topic', 'oakd/rgb/preview/image_raw',
                              'Camera image topic')
        colors = declare(self, 'target_colors', ['red', 'blue', 'green'],
                         f'Cone colors to accept, each with an hsv.<color> range; known: '
                         f'{list(DEFAULT_HSV)}')
        self._targets = {}
        for color in colors:
            if color not in DEFAULT_HSV:
                raise ValueError(f'No HSV range for {color}; known: {list(DEFAULT_HSV)}')
            self._targets[color] = parse_ranges(declare(
                self, f'hsv.{color}', DEFAULT_HSV[color],
                'HSV ranges, six numbers each: h_lo s_lo v_lo h_hi s_hi v_hi (H 0-179, S and '
                'V 0-255); red needs two'))
        d = DetectionSettings()
        self._settings = DetectionSettings(
            open_kernel=declare(self, 'open_kernel', d.open_kernel,
                                'Opening kernel size (px), removes specks', 1, 31),
            close_kernel=declare(self, 'close_kernel', d.close_kernel,
                                 'Closing kernel size (px), fills holes', 1, 31),
            min_area=declare(self, 'min_contour_area', d.min_area,
                             'Smaller contours (px) are noise', 1, 100000),
            min_aspect=declare(self, 'min_aspect', d.min_aspect,
                               'Smallest cone height / width', 0.05, 10.0),
            max_aspect=declare(self, 'max_aspect', d.max_aspect,
                               'Largest cone height / width', 0.05, 10.0),
            min_taper=declare(self, 'min_taper', d.min_taper,
                              'Smallest 1 - top width / bottom width', 0.0, 1.0),
            min_symmetry=declare(self, 'min_symmetry', d.min_symmetry,
                                 'Smallest left-right symmetry (0-1)', 0.0, 1.0),
            min_solidity=declare(self, 'min_solidity', d.min_solidity,
                                 'Smallest contour area / convex hull area', 0.0, 1.0),
            max_vertices=declare(self, 'max_vertices', d.max_vertices,
                                 'Most corners of the simplified outline', 3, 100),
            approx_epsilon=declare(self, 'approx_epsilon', d.approx_epsilon,
                                   'approxPolyDP tolerance, as a fraction of the perimeter',
                                   0.001, 0.5),
            min_color_fraction=declare(self, 'min_color_fraction', d.min_color_fraction,
                                       'Smallest fraction of strongly colored pixels around '
                                       'a cone that are of its color', 0.0, 1.0),
            color_context=declare(self, 'color_context', d.color_context,
                                  'How far beyond its box (fraction of its size) the color '
                                  'check looks', 0.0, 2.0),
            edge_margin=declare(self, 'edge_margin', d.edge_margin,
                                'Reject boxes this close (px) to the image edge', 0, 100),
            weights={name: declare(self, f'weights.{name}', weight,
                                   f'Weight of the {name} score in the confidence', 0.0, 10.0)
                     for name, weight in DEFAULT_WEIGHTS.items()},
            min_confidence=declare(self, 'min_confidence', d.min_confidence,
                                   'Smallest confidence (0-1) of an accepted cone', 0.0, 1.0))
        if self._settings.min_aspect > self._settings.max_aspect:
            raise ValueError('min_aspect is above max_aspect')
        if sum(self._settings.weights.values()) <= 0.0:
            raise ValueError('at least one weight must be positive')

        self._bridge = CvBridge()
        self._detections_pub = self.create_publisher(Detection2DArray, 'cone_detections', 10)
        self._debug_pub = self.create_publisher(Image, 'cone_detections/debug', 1)
        self._mask_pub = self.create_publisher(Image, 'cone_detections/mask', 1)
        self.create_subscription(Image, image_topic, self._image_callback,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'Looking for {", ".join(self._targets)} cones in {image_topic}')

    def _image_callback(self, msg):
        """Find the cones in an image and publish them, plus the debug images if watched."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Cannot convert the image: {e}', throttle_duration_sec=5.0)
            return
        candidates = detect_cones(bgr, self._targets, self._settings)
        out = Detection2DArray()
        out.header = msg.header
        for c in candidates:
            if not c.accepted():
                continue
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = c.u
            detection.bbox.center.position.y = c.v
            detection.bbox.size_x = float(c.right - c.left)
            detection.bbox.size_y = float(c.bottom - c.top)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = c.color
            hypothesis.hypothesis.score = c.confidence
            detection.results.append(hypothesis)
            out.detections.append(detection)
        self._detections_pub.publish(out)
        if self._debug_pub.get_subscription_count() > 0:
            debug = self._bridge.cv2_to_imgmsg(draw_candidates(bgr, candidates), 'bgr8')
            debug.header = msg.header
            self._debug_pub.publish(debug)
        if self._mask_pub.get_subscription_count() > 0:
            mask = self._bridge.cv2_to_imgmsg(target_mask(bgr, self._targets, self._settings),
                                              'mono8')
            mask.header = msg.header
            self._mask_pub.publish(mask)


def main(args=None):
    """Run the Cone Detector node until Ctrl-C."""
    rclpy.init(args=args)
    try:
        node = ConeDetector()
    except Exception as e:  # bad parameters
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

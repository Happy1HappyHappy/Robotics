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

"""Tests of the parameters module (needs rclpy; skipped where ROS is not installed)."""

import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def make_node():
    """Return a function that creates a node with parameter overrides, cleaned up after."""
    from rclpy.node import Node
    rclpy.init()
    nodes = []

    def make(overrides=()):
        node = Node('test_parameters', parameter_overrides=list(overrides))
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def test_float_parameter_gets_description_range_and_read_only(make_node):
    from cs5335_cone_navigation.parameters import declare
    node = make_node()
    assert declare(node, 'speed', 0.5, 'Top speed (m/s)', 0.0, 1.0) == 0.5
    descriptor = node.describe_parameter('speed')
    assert descriptor.description == 'Top speed (m/s)'
    assert descriptor.read_only
    assert descriptor.floating_point_range[0].to_value == 1.0


def test_integer_and_string_parameters(make_node):
    from cs5335_cone_navigation.parameters import declare
    node = make_node()
    assert declare(node, 'count', 3, 'How many', 1, 10) == 3
    assert node.describe_parameter('count').integer_range[0].from_value == 1
    assert declare(node, 'topic', 'scan', 'A topic') == 'scan'
    assert not node.describe_parameter('topic').floating_point_range


def test_value_outside_the_range_is_refused(make_node):
    from cs5335_cone_navigation.parameters import declare
    from rclpy.exceptions import InvalidParameterValueException
    from rclpy.parameter import Parameter
    node = make_node([Parameter('speed', value=2.0)])
    with pytest.raises(InvalidParameterValueException):
        declare(node, 'speed', 0.5, 'Top speed (m/s)', 0.0, 1.0)

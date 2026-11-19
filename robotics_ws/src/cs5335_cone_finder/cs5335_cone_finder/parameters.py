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

"""Declare node parameters with a description and, for numbers, an allowed range."""

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


def declare(node, name, default, description, low=None, high=None):
    """
    Declare a read-only parameter on a node and return its value.

    description says what the parameter is and its unit. For a float or integer, low and
    high (if given) are the allowed range; a value outside it is refused when the node
    starts. Read-only means the value is set when the node starts (from a parameter file
    or the command line) and cannot change while it runs, since the nodes read their
    parameters only once.
    """
    descriptor = ParameterDescriptor(description=description, read_only=True)
    if low is not None and high is not None:
        if isinstance(default, float):
            descriptor.floating_point_range = [
                FloatingPointRange(from_value=float(low), to_value=float(high), step=0.0)]
        elif isinstance(default, int) and not isinstance(default, bool):
            descriptor.integer_range = [
                IntegerRange(from_value=int(low), to_value=int(high), step=1)]
    return node.declare_parameter(name, default, descriptor).value

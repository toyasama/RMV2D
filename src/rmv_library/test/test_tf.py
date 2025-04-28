# MIT License
#
# Copyright (c) 2025 [Tognama Emery-donald]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import pytest
from unittest.mock import MagicMock, call
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rmv_library.tf_management.tf import TFManager


@pytest.fixture
def mock_node():
    node = MagicMock()
    node.create_subscription = MagicMock()
    node.get_logger().info = MagicMock()
    return node


@pytest.fixture
def mock_parameters():
    params = MagicMock()
    params.frames.main_frame = "world"
    return params


def makeTfMessage(num=1):
    tf_msg = TFMessage()
    for i in range(num):
        tf = TransformStamped()
        tf.header.frame_id = "world"
        tf.child_frame_id = f"child_{i}"
        tf.transform.translation.x = float(i)
        tf.transform.rotation.w = 1.0
        tf_msg.transforms.append(tf)
    return tf_msg


@pytest.mark.timeout(5)
def testInitializationSubscriptions(mock_node, mock_parameters):
    manager = TFManager(mock_node, mock_parameters)

    assert mock_node.create_subscription.call_count == 2

    mock_node.get_logger().info.assert_called_with(
        "TFManager initialized successfully."
    )

    assert isinstance(
        manager.transform_graph._graph, type(manager._transform_graph._graph)
    )


@pytest.mark.timeout(5)
def testTfCallbackAddsDynamicTransforms(mock_node, mock_parameters):
    manager = TFManager(mock_node, mock_parameters)
    mock_graph = MagicMock()
    manager._transform_graph = mock_graph

    tf_msg = makeTfMessage(num=3)
    manager._tfCallback(tf_msg)

    calls = [call(t, static=False) for t in tf_msg.transforms]
    mock_graph.addTransform.assert_has_calls(calls, any_order=True)
    assert mock_graph.addTransform.call_count == 3


@pytest.mark.timeout(5)
def testTfStaticCallbackAddsStaticTransforms(mock_node, mock_parameters):
    manager = TFManager(mock_node, mock_parameters)
    mock_graph = MagicMock()
    manager._transform_graph = mock_graph

    tf_msg = makeTfMessage(num=2)
    manager._tfStaticCallback(tf_msg)

    calls = [call(t, static=True) for t in tf_msg.transforms]
    mock_graph.addTransform.assert_has_calls(calls, any_order=True)
    assert mock_graph.addTransform.call_count == 2

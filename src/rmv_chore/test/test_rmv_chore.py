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
from unittest.mock import MagicMock, patch
from rmv_chore.rmv_chore_node import RMVChoreNode
from rmv_library import MarkerRmv
import rclpy
from geometry_msgs.msg import TransformStamped, Pose, Vector3, Point
from builtin_interfaces.msg import Time
import time
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
import copy


@pytest.fixture
def future_time():
    t = Time()
    t.sec = 20
    t.nanosec = 0
    return t


@pytest.fixture
def pose():
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    return pose


@pytest.fixture
def marker(pose):
    marker = Marker()
    marker.id = 1
    marker.ns = "test_ns"
    marker.pose = pose
    marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    marker.lifetime.sec = 5
    marker.lifetime.nanosec = 0
    marker.header = Header(frame_id="world")
    marker.points = [Point(x=1.0, y=2.0, z=3.0)]
    marker.type = Marker.CUBE
    return marker


@pytest.fixture(scope="session", autouse=True)
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def rmv_chore_node():
    with patch("rclpy.node.Node.create_timer"), patch("rclpy.node.Node.get_logger"):
        node = RMVChoreNode()
        yield node
        if not node.destroyed:
            node.stop()


@pytest.mark.timeout(5)
def testInitializationAndStop(rmv_chore_node):
    assert isinstance(rmv_chore_node, RMVChoreNode)
    assert rmv_chore_node.destroyed is False
    rmv_chore_node.stop()
    assert rmv_chore_node.destroyed is True


@pytest.mark.timeout(5)
def testProjectToMainFrameWithMainFrameMarker(rmv_chore_node):
    marker = MagicMock()
    marker.frame_id = rmv_chore_node.tf_manager.transform_graph.main_frame
    marker.pose = MagicMock()
    result = rmv_chore_node.projectToMainFrame([marker], [])
    assert len(result) == 0


@pytest.mark.timeout(5)
def testProjectToMainFrameWithTransform(rmv_chore_node, marker):
    import copy

    marker_2 = copy.deepcopy(marker)
    marker_2.header.frame_id = "child"

    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = "world"
    transform_stamped.child_frame_id = "child"
    transform_stamped.transform.translation.x = 1.0
    transform_stamped.transform.translation.y = 2.0
    transform_stamped.transform.translation.z = 3.0
    transform_stamped.transform.rotation.w = 1.0

    rmv_chore_node.parameters.frames.main_frame = "world"

    time.sleep(1.0)
    rmv_chore_node.tf_manager.transform_graph.addTransform(
        transform_stamped, static=True
    )
    time.sleep(1.0)

    drawer_infos = (
        rmv_chore_node.tf_manager.transform_graph.getTransformsFromMainFrame()
    )

    markers = [MarkerRmv(marker, Time()), MarkerRmv(marker_2, Time())]
    result = rmv_chore_node.projectToMainFrame(markers, drawer_infos)

    assert len(result) == 2
    assert result[0].modified_pose is not None
    assert result[1].modified_pose is not True

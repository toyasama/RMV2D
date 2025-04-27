import pytest
from unittest.mock import MagicMock
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
from rmv_library.markers_management.marker_handler import MarkersHandler
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3
import time
from builtin_interfaces.msg import Duration


@pytest.fixture
def mock_node():
    class MockClock:
        def now(self):
            t = time.monotonic()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            return MagicMock(
                to_msg=MagicMock(return_value=Time(sec=sec, nanosec=nanosec))
            )

    mock_node = MagicMock()
    mock_node.get_clock.return_value = MockClock()
    return mock_node


@pytest.fixture
def sample_marker():
    marker = Marker()
    marker.ns = "test_ns"
    marker.id = 1
    marker.pose = Pose()
    marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    marker.header = Header(frame_id="map")
    marker.lifetime = Duration(sec=1, nanosec=0)
    return marker


@pytest.fixture
def expired_marker():
    marker = Marker()
    marker.ns = "expired"
    marker.id = 2
    marker.pose = Pose()
    marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    marker.header = Header(frame_id="world")
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


@pytest.mark.timeout(5)
def testAddMarkerStoresNonExpiredMarker(mock_node, sample_marker):
    handler = MarkersHandler()
    handler.addMarker(sample_marker)
    time.sleep(1.0)
    assert len(handler.markers) == 1
    marker = handler.markers[0]
    assert marker.identifier == (sample_marker.ns, sample_marker.id)
    handler.stop()


@pytest.mark.timeout(5)
def testExpiredMarkerIsNotStored(mock_node, expired_marker):
    handler = MarkersHandler()
    handler.addMarker(expired_marker)
    time.sleep(1.0)
    assert len(handler.markers) == 0
    handler.stop()


@pytest.mark.timeout(5)
def testClearMarkersList(mock_node, sample_marker):
    handler = MarkersHandler()
    handler.addMarker(sample_marker)
    time.sleep(1.0)
    assert len(handler.markers) == 1
    handler.clearMarkersList()
    assert len(handler.markers) == 0
    handler.stop()


@pytest.mark.timeout(5)
def testAddMarkerArrayStoresMarkers(mock_node, sample_marker):
    marker_array = MarkerArray()
    marker_array.markers.append(sample_marker)

    handler = MarkersHandler()
    handler.addMarker(marker_array)
    time.sleep(1.0)
    assert len(handler.markers) == 1
    marker = handler.markers[0]
    assert marker.identifier == (sample_marker.ns, sample_marker.id)
    handler.stop()


@pytest.mark.timeout(5)
def testAddMultipleMarkers(mock_node, sample_marker):
    handler = MarkersHandler()

    markers = []
    for i in range(5):
        marker = Marker()
        marker.ns = "test_ns"
        marker.id = i
        marker.pose = sample_marker.pose
        marker.scale = sample_marker.scale
        marker.color = sample_marker.color
        marker.header = sample_marker.header
        marker.lifetime = sample_marker.lifetime
        markers.append(marker)

    for marker in markers:
        handler.addMarker(marker)

    time.sleep(0.1)
    assert len(handler.markers) == 5

    ids = {(marker.identifier) for marker in handler.markers}
    expected_ids = {("test_ns", i) for i in range(5)}
    assert ids == expected_ids
    handler.stop()


@pytest.mark.timeout(5)
def testMergeIdenticalMarkersKeepsLast(mock_node):
    handler = MarkersHandler()

    marker1 = Marker()
    marker1.ns = "merge_ns"
    marker1.id = 42
    marker1.pose.position.x = 1.0
    marker1.scale.x = 1.0
    marker1.color.r = 1.0
    marker1.header.frame_id = "map"
    marker1.lifetime.sec = 5

    marker2 = Marker()
    marker2.ns = "merge_ns"
    marker2.id = 42
    marker2.pose.position.x = 2.0
    marker2.scale.x = 2.0
    marker2.color.r = 0.5
    marker2.header.frame_id = "world"
    marker2.lifetime.sec = 10

    handler.addMarker(marker1)
    time.sleep(0.05)
    handler.addMarker(marker2)
    time.sleep(0.1)

    assert len(handler.markers) == 1

    merged_marker = handler.markers[0]
    assert merged_marker.pose.position.x == 2.0
    assert merged_marker.scale.x == 2.0
    assert merged_marker.color.r == 0.5
    assert merged_marker.frame_id == "world"
    assert merged_marker.lifetime.sec == 10
    handler.stop()

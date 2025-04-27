import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
from rmv_library.markers_management.markers import MarkerRmv
import copy


@pytest.fixture
def time():
    t = Time()
    t.sec = 10
    t.nanosec = 0
    return t


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
    marker.header = Header(frame_id="base_link")
    marker.points = [Point(x=1.0, y=2.0, z=3.0)]
    marker.type = Marker.CUBE
    return marker


@pytest.mark.timeout(5)
def testMarkerRmvProperties(marker, time):
    rmv = MarkerRmv(marker, time)

    assert rmv.identifier == (marker.ns, marker.id)
    assert rmv.pose == marker.pose
    assert rmv.scale == marker.scale
    assert rmv.color == marker.color
    assert rmv.lifetime == marker.lifetime
    assert rmv.frame_id == marker.header.frame_id
    assert rmv.points == marker.points
    assert rmv.type == marker.type


@pytest.mark.timeout(5)
def testFrameIdSetter(marker, time):
    rmv = MarkerRmv(marker, time)
    rmv.frame_id = "map"
    assert rmv.frame_id == "map"


@pytest.mark.timeout(5)
def testModifiedPose(marker, time, pose):
    rmv = MarkerRmv(marker, time)

    assert rmv.modified_pose == marker.pose

    new_pose = Pose()
    new_pose.position.x = 99.0
    rmv.modified_pose = new_pose

    assert rmv.modified_pose == new_pose


@pytest.mark.timeout(5)
def testGetTransform(marker, time):
    rmv = MarkerRmv(marker, time)
    assert rmv.pose == marker.pose


@pytest.mark.timeout(5)
def testIsExpiredTrue(marker, time, future_time):
    rmv = MarkerRmv(marker, time)
    assert rmv.isExpired(future_time)


@pytest.mark.timeout(5)
def testIsExpiredFalse(marker, time):
    still_valid = Time(sec=time.sec + 3, nanosec=0)
    rmv = MarkerRmv(marker, time)
    assert not rmv.isExpired(still_valid)


@pytest.mark.timeout(5)
def testMarkerRmvEquality(marker, time):
    rmv1 = MarkerRmv(marker, time)
    rmv2 = MarkerRmv(marker, time)

    assert rmv1 == rmv2

    marker_copy = copy.deepcopy(marker)
    marker_copy.id = 99
    rmv3 = MarkerRmv(marker_copy, time)

    assert rmv1 != rmv3

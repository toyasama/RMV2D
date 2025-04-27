import pytest
import networkx as nx
from unittest.mock import MagicMock
from geometry_msgs.msg import TransformStamped, Transform

from rmv_library.tf_management.graph import TransformGraph
from rmv_library.tf_management.transform_rmv import RmvTransform
from tf_transformations import quaternion_from_euler
import time


@pytest.fixture
def dummy_params():
    params = MagicMock()
    params.frames.main_frame = "world"
    return params


@pytest.fixture
def basic_transform():
    t = TransformStamped()
    t.header.frame_id = "world"
    t.child_frame_id = "camera"
    t.transform.translation.x = 1.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


def makeTransform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    """Helper to create a TransformStamped with translation and simple yaw rotation."""
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    q = quaternion_from_euler(0.0, 0.0, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t


@pytest.mark.timeout(5)
def testAddAndGetTransform(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)
    graph.addTransform(basic_transform)

    tf = graph.getTransform("world", "camera")
    assert isinstance(tf, Transform)
    assert tf.translation.x == 1.0
    graph.stop()


@pytest.mark.timeout(5)
def testInverseTransformMsg(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)
    inv = graph._inverseTransformMsg(basic_transform)
    assert inv.header.frame_id == "camera"
    assert inv.child_frame_id == "world"
    graph.stop()


@pytest.mark.timeout(5)
def testFramesAndSubframes(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)
    graph.addTransform(basic_transform)

    assert "world" in graph.frames
    assert "camera" in graph.frames
    assert graph.main_frame == "world"
    assert "camera" in graph.sub_frames
    assert "world" not in graph.sub_frames
    graph.stop()


@pytest.mark.timeout(5)
def testComputeTransformInfo(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)
    graph.addTransform(basic_transform)

    next_transform = TransformStamped()
    next_transform.header.frame_id = "camera"
    next_transform.child_frame_id = "lidar"
    next_transform.transform.translation.x = 2.0
    next_transform.transform.translation.y = 1.0
    next_transform.transform.rotation.w = 1.0
    graph.addTransform(next_transform)

    path = ["world", "camera", "lidar"]
    result = graph._computeTransformInfo(path)
    assert isinstance(result, Transform)
    assert result.translation.x == 3.0
    assert result.translation.y == 1.0
    graph.stop()


@pytest.mark.timeout(5)
def testComplexComputeTransformInfo(dummy_params):
    graph = TransformGraph(dummy_params)

    transforms = [
        makeTransform("world", "base", x=1.0),
        makeTransform("base", "camera", x=2.0),
        makeTransform("camera", "lidar", x=3.0),
        makeTransform("lidar", "imu", x=4.0),
        makeTransform("imu", "gps", x=5.0),
        makeTransform("world", "gps", x=6.0),
        makeTransform("world", "wheel_left", x=6.0),
    ]

    for tf in transforms:
        graph.addTransform(tf)

    path = ["world", "base", "camera", "lidar", "imu"]
    result = graph._computeTransformInfo(path)

    assert isinstance(result, Transform)
    assert result.translation.x == pytest.approx(10.0)
    assert result.translation.y == pytest.approx(0.0)
    assert result.translation.z == pytest.approx(0.0)
    graph.stop()


@pytest.mark.timeout(5)
def testRemoveExpiredEdges(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)

    expired = MagicMock(spec=RmvTransform)
    expired.isExpired = True
    expired.name = "camera"
    expired.transform = basic_transform.transform
    expired.drawer_info.main_frame = "world"
    expired._initial_direction = True

    with graph._graph_lock:
        graph._graph.add_edge("world", "camera", frameInfo=expired)
        graph._graph.add_edge("camera", "world", frameInfo=expired)

    graph._removeExpiredEdges()
    assert not graph._graph.has_edge("world", "camera")
    graph.stop()


@pytest.mark.timeout(5)
def testGetTransformsFromMainFrame(dummy_params, basic_transform):
    graph = TransformGraph(dummy_params)
    graph.addTransform(basic_transform)
    time.sleep(0.5)
    results = graph.getTransformsFromMainFrame()
    assert isinstance(results, list)
    assert len(results) > 0
    graph.stop()


@pytest.mark.timeout(5)
def testStaticTransformNeverExpires(dummy_params):
    graph = TransformGraph(dummy_params)

    tf_static = makeTransform("world", "camera")
    graph.addTransform(tf_static, static=True)

    time.sleep(3.2)
    graph._removeExpiredEdges()

    assert ("world", "camera") in graph._graph.edges
    assert ("camera", "world") in graph._graph.edges
    graph.stop()


@pytest.mark.timeout(5)
def testDynamicTransformExpires(dummy_params):
    graph = TransformGraph(dummy_params)

    tf_dynamic = makeTransform("world", "moving_frame")
    graph.addTransform(tf_dynamic, static=False)

    assert ("world", "moving_frame") in graph._graph.edges

    time.sleep(3.2)
    graph._removeExpiredEdges()

    assert ("world", "moving_frame") not in graph._graph.edges
    assert ("moving_frame", "world") not in graph._graph.edges
    graph.stop()


@pytest.mark.timeout(5)
def testMixedStaticAndDynamic(dummy_params):
    graph = TransformGraph(dummy_params)

    tf_static = makeTransform("world", "static_frame")
    tf_dynamic = makeTransform("world", "moving_frame")

    graph.addTransform(tf_static, static=True)
    graph.addTransform(tf_dynamic, static=False)

    time.sleep(3.2)
    graph._removeExpiredEdges()

    assert ("world", "static_frame") in graph._graph.edges
    assert ("world", "moving_frame") not in graph._graph.edges
    graph.stop()

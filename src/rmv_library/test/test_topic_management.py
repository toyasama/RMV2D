import pytest
from unittest.mock import MagicMock

from rmv_library.topic_management.topic_manager import TopicManager


@pytest.fixture
def mockNode():
    node = MagicMock()
    node.create_timer = MagicMock()
    node.get_logger = MagicMock()
    node.get_topic_names_and_types = MagicMock()
    node.count_publishers = MagicMock()
    return node


@pytest.fixture
def mockMarkerHandler():
    handler = MagicMock()
    handler.addMarker = MagicMock()
    return handler


@pytest.fixture
def topicManager(mockNode, mockMarkerHandler):
    return TopicManager(mockNode, mockMarkerHandler)


def testInitialization(topicManager, mockNode):
    mockNode.create_timer.assert_called_once()
    mockNode.get_logger().info.assert_called_with("TopicManager created successfully")


def testHasMatchingTypeMatch(topicManager):
    received = ["std_msgs/msg/String", "visualization_msgs/msg/Marker"]
    expected = ["visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"]
    match = topicManager._hasMatchingType(received, expected)
    assert match == "visualization_msgs/msg/Marker"


def testHasMatchingTypeNoMatch(topicManager):
    received = ["std_msgs/msg/String", "geometry_msgs/msg/Pose"]
    expected = ["visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"]
    match = topicManager._hasMatchingType(received, expected)
    assert match is None


def testHasPublisherTrue(topicManager, mockNode):
    mockNode.count_publishers.return_value = 1
    assert topicManager._hasPublisher("some_topic") is True


def testHasPublisherFalse(topicManager, mockNode):
    mockNode.count_publishers.return_value = 0
    assert topicManager._hasPublisher("some_topic") is False


def testFilterTopicsWithMatchingAndPublishers(topicManager, mockNode):
    topics = [
        ("topic1", ["visualization_msgs/msg/Marker"]),
        ("topic2", ["std_msgs/msg/String"]),
        ("topic3", ["visualization_msgs/msg/MarkerArray"]),
    ]
    expected_types = [
        "visualization_msgs/msg/Marker",
        "visualization_msgs/msg/MarkerArray",
    ]

    def count_publishers_side_effect(topic_name):
        return 1 if topic_name != "topic2" else 0

    mockNode.count_publishers.side_effect = count_publishers_side_effect

    filtered = topicManager._filter(topics, expected_types)
    assert filtered == [
        ("topic1", "visualization_msgs/msg/Marker"),
        ("topic3", "visualization_msgs/msg/MarkerArray"),
    ]


def testSubscribeToTopics(topicManager):
    topicManager.subscribe = MagicMock()
    topics = [
        ("topic1", "visualization_msgs/msg/Marker"),
        ("topic2", "visualization_msgs/msg/MarkerArray"),
    ]
    topicManager._subscribeToTopics(topics)
    topicManager.subscribe.assert_any_call("topic1", "visualization_msgs/msg/Marker")
    topicManager.subscribe.assert_any_call(
        "topic2", "visualization_msgs/msg/MarkerArray"
    )
    assert topicManager.subscribe.call_count == 2


def testRemoveUnpublishedTopics(topicManager):
    # Simuler l'état actuel
    topicManager._SubscriptionManager__subscriptions = {
        "topic1": MagicMock(),
        "topic2": MagicMock(),
        "topic3": MagicMock(),
    }
    topicManager.unsubscribe = MagicMock()

    # Seulement topic1 et topic3 existent encore
    filtered = [("topic1", "visualization_msgs/msg/Marker")]

    topicManager._removeUnpublishedTopics(filtered)
    topicManager.unsubscribe.assert_any_call("topic2")
    topicManager.unsubscribe.assert_any_call("topic3")
    assert topicManager.unsubscribe.call_count == 2


def testFindMarkersTopicsCallsAllInternals(topicManager, mockNode):
    topicManager._filter = MagicMock(
        return_value=[("topic1", "visualization_msgs/msg/Marker")]
    )
    topicManager._subscribeToTopics = MagicMock()
    topicManager._removeUnpublishedTopics = MagicMock()

    mockNode.get_topic_names_and_types.return_value = [
        ("topic1", ["visualization_msgs/msg/Marker"]),
        ("topic2", ["std_msgs/msg/String"]),
    ]

    topicManager._TopicManager__findMarkersTopics()

    topicManager._filter.assert_called_once()
    topicManager._subscribeToTopics.assert_called_once()
    topicManager._removeUnpublishedTopics.assert_called_once()

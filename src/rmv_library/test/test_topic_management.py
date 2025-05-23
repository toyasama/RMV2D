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


@pytest.mark.timeout(5)
def testInitialization(topicManager, mockNode):
    mockNode.create_timer.assert_called_once()
    mockNode.get_logger().info.assert_called_with("TopicManager created successfully")


@pytest.mark.timeout(5)
def testHasMatchingTypeMatch(topicManager):
    received = ["std_msgs/msg/String", "visualization_msgs/msg/Marker"]
    expected = ["visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"]
    match = topicManager._hasMatchingType(received, expected)
    assert match == "visualization_msgs/msg/Marker"


@pytest.mark.timeout(5)
def testHasMatchingTypeNoMatch(topicManager):
    received = ["std_msgs/msg/String", "geometry_msgs/msg/Pose"]
    expected = ["visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"]
    match = topicManager._hasMatchingType(received, expected)
    assert match is None


@pytest.mark.timeout(5)
def testHasPublisherTrue(topicManager, mockNode):
    mockNode.count_publishers.return_value = 1
    assert topicManager._hasPublisher("some_topic") is True


@pytest.mark.timeout(5)
def testHasPublisherFalse(topicManager, mockNode):
    mockNode.count_publishers.return_value = 0
    assert topicManager._hasPublisher("some_topic") is False


@pytest.mark.timeout(5)
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


@pytest.mark.timeout(5)
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


@pytest.mark.timeout(5)
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


@pytest.mark.timeout(5)
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

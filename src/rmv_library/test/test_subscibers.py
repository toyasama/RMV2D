import pytest
from unittest.mock import MagicMock

from visualization_msgs.msg import Marker, MarkerArray
from rmv_library.topic_management.subscription_manager import SubscriptionManager


@pytest.fixture
def mockNode():
    node = MagicMock()
    node.create_subscription = MagicMock()
    node.get_logger = MagicMock()
    return node


@pytest.fixture
def mockMarkerHandler():
    handler = MagicMock()
    handler.addMarker = MagicMock()
    return handler


@pytest.fixture
def subscriptionManager(mockNode, mockMarkerHandler):
    return SubscriptionManager(mockNode, mockMarkerHandler)


def testSubscribeAddsNewSubscription(subscriptionManager, mockNode):
    subscriptionManager.subscribe("topic1", "visualization_msgs/msg/Marker")
    assert "topic1" in subscriptionManager.active_topics
    mockNode.create_subscription.assert_called_once()


def testSubscribeDoesNotDuplicate(subscriptionManager, mockNode):
    subscriptionManager.subscribe("topic1", "visualization_msgs/msg/Marker")
    subscriptionManager.subscribe("topic1", "visualization_msgs/msg/Marker")
    assert subscriptionManager.active_topics.count("topic1") == 1
    assert mockNode.create_subscription.call_count == 1


def testUnsubscribeRemovesTopic(subscriptionManager, mockNode):
    subscriptionManager.subscribe("topic1", "visualization_msgs/msg/Marker")
    subscriptionManager.unsubscribe("topic1")
    assert "topic1" not in subscriptionManager.active_topics


def testUnsubscribeNonExistentTopic(subscriptionManager, mockNode):
    initial_topics = subscriptionManager.active_topics.copy()
    subscriptionManager.unsubscribe("non_existent_topic")
    assert subscriptionManager.active_topics == initial_topics


def testCallbackAddsMarker(subscriptionManager, mockMarkerHandler):
    marker = Marker()
    subscriptionManager.callback(marker)
    mockMarkerHandler.addMarker.assert_called_once_with(marker)


def testGetMessageTypeValidMarker(subscriptionManager):
    messageType = subscriptionManager._getMessageType("visualization_msgs/msg/Marker")
    assert messageType == Marker


def testGetMessageTypeValidMarkerArray(subscriptionManager):
    messageType = subscriptionManager._getMessageType(
        "visualization_msgs/msg/MarkerArray"
    )
    assert messageType == MarkerArray


def testGetMessageTypeInvalid(subscriptionManager):
    with pytest.raises(ValueError, match="Unsupported type: InvalidType"):
        subscriptionManager._getMessageType("visualization_msgs/msg/InvalidType")


def testFullSubscribeAndUnsubscribeFlow(subscriptionManager, mockNode):
    subscriptionManager.subscribe("topic1", "visualization_msgs/msg/Marker")
    subscriptionManager.subscribe("topic2", "visualization_msgs/msg/MarkerArray")

    assert set(subscriptionManager.active_topics) == {"topic1", "topic2"}
    subscriptionManager.unsubscribe("topic1")
    assert subscriptionManager.active_topics == ["topic2"]

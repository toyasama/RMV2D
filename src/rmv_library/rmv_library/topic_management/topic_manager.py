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


from rclpy.node import Node

from ..markers_management.marker_handler import MarkersHandler
from .subscription_manager import SubscriptionManager


class TopicManager(SubscriptionManager):
    def __init__(self, node: Node, markers_handler: MarkersHandler) -> None:
        """Constructor for the TopicManager class.

        Args:
            node (Node): The node object.

        """
        SubscriptionManager.__init__(self, node, markers_handler)
        node.create_timer(0.1, self.__findMarkersTopicsCallBack)
        node.get_logger().info("TopicManager created successfully")

    def __findMarkersTopicsCallBack(self) -> None:
        """Discover topics related to markers, subscribe to new ones, and remove obsolete subscriptions."""
        self.__findMarkersTopics()

    def __findMarkersTopics(self):
        """Discover topics related to markers, subscribe to new ones, and remove obsolete subscriptions.
        This method will be decorated to log execution time.
        """
        topics_and_types = self._node.get_topic_names_and_types()
        expected_types = [
            "visualization_msgs/msg/Marker",
            "visualization_msgs/msg/MarkerArray",
        ]
        filtered_topics = self._filter(topics_and_types, expected_types)

        self._removeUnpublishedTopics(filtered_topics)
        self._subscribeToTopics(filtered_topics)

    def _filter(
        self,
        topics: list[tuple[str, list[str]]],
        expected_types: list[str],
    ) -> list[tuple[str, str]]:
        """Filter topics based on expected types and active publishers.

        Args:
            topics (list): A list of tuples containing topic names and their types.
            expected_types (list): A list of expected message types.

        Returns:
            list: A list of tuples with topic names and matching types.

        """
        return [
            (topic, matching_type)
            for topic, types in topics
            if (matching_type := self._hasMatchingType(types, expected_types))
            and self._hasPublisher(topic)
        ]

    def _subscribeToTopics(self, filtered_topics: list[tuple[str, str]]) -> None:
        """Subscribe to topics related to markers.

        Args:
            filtered_topics (list): The list of currently available marker topics.

        """
        for topic, topic_type in filtered_topics:
            self.subscribe(topic, topic_type)

    def _removeUnpublishedTopics(self, filtered_topics: list[tuple[str, str]]) -> None:
        """Remove subscriptions to topics that are no longer published or have no active publishers.

        Args:
            filtered_topics (list): The list of currently available marker topics.

        """
        activeTopics = {topic for topic, _ in filtered_topics}
        for subscribed_topic in self.active_topics:
            if subscribed_topic not in activeTopics:
                self.unsubscribe(subscribed_topic)

    def _hasMatchingType(
        self, received_types: list[str], expected_types: list[str]
    ) -> str | None:
        """Check if any of the received types match one of the expected types.

        Args:
            received_types (list): A list of types associated with a topic.
            expected_types (list): A list of expected types.

        Returns:
            str | None: The matching type if found, otherwise None.

        """
        return next((t for t in received_types if t in expected_types), None)

    def _hasPublisher(self, topic: str) -> bool:
        """Check if a topic has active publishers.

        Args:
            topic (str): The topic name to check.

        Returns:
            bool: True if the topic has at least one publisher, False otherwise.

        """
        return self._node.count_publishers(topic) > 0

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from visualization_msgs.msg import Marker, MarkerArray

from ..markers_management.marker_handler import MarkersHandler


class SubscriptionManager:
    def __init__(self, node: Node, markers_handler: MarkersHandler):
        """Manager for handling topic __subscriptions.

        Args:
            node (Node): The ROS2 node object.

        """
        self._node = node
        self.__subscriptions: dict[str, Subscription] = {}
        self.marker_handler = markers_handler

    @property
    def active_topics(self) -> list[str]:
        """Get a list of currently active topics.

        Returns:
            list: List of topic names.

        """
        return list(self.__subscriptions.keys())

    def subscribe(self, topic: str, topic_type: str):
        """Subscribe to a new topic if not already subscribed.

        Args:
            topic (str): Topic name.
            topic_type (str): ROS2 message type string.
            callback: Callback function for the topic.

        """
        if topic not in list(self.__subscriptions.keys()):
            print(f"Subscribe to {topic}")
            message_type = self._getMessageType(topic_type)
            self.__subscriptions[topic] = self._node.create_subscription(
                message_type, topic, self.callback, qos_profile_sensor_data
            )

    def callback(self, message: Marker | MarkerArray):
        """Callback function for the subscribed topic.

        Args:
            message (Marker | MarkerArray): The received message.

        """
        self.marker_handler.addMarker(message)
        # self.timer_logger.logExecutionTime(self.addMarker)(message)

    def unsubscribe(self, topic: str):
        """Unsubscribe from a topic.

        Args:
            topic (str): Topic name.

        """
        if topic in self.__subscriptions:
            self._node.get_logger().info(f"Unsubscribing from: {topic}")
            del self.__subscriptions[topic]

    def _getMessageType(self, type_string: str) -> type[Marker | MarkerArray]:
        """Get the ROS2 message class based on the type string.

        Args:
            type_string (str): The ROS2 type string (e.g., "visualization_msgs/msg/Marker").

        Returns:
            Type[Marker | MarkerArray]: The corresponding message class.

        Raises:
            ValueError: If the type string is invalid or unsupported.

        """
        message_type_map = {
            "Marker": Marker,
            "MarkerArray": MarkerArray,
        }
        message_type = type_string.split("/")[-1]
        if message_type in message_type_map:
            return message_type_map[message_type]
        raise ValueError(f"Unsupported type: {message_type}")

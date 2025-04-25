from visualization_msgs.msg import Marker, MarkerArray
from .markers import MarkerRmv
from builtin_interfaces.msg import Time
from abc import ABC, abstractmethod


class BaseMessage(ABC):
    """Base class for message processing.
    Convert a message of type Marker or MarkerArray into one or a list of MarkerRmv.
    Args:
        message_type (type): The type of message to process (Marker or MarkerArray).
    """

    def __init__(self, message_type: type[Marker | MarkerArray]):
        self.message_type = message_type

    @abstractmethod
    def process(self, message, time: Time):
        """Do something with the message."""


class MarkerMessage(BaseMessage):
    """Class to process a single Marker message.
    Convert a message of type Marker into a MarkerRmv.
    Args:
        message_type (type): The type of message to process (Marker).
    """

    def __init__(self):
        super().__init__(Marker)

    @staticmethod
    def process(message: Marker, time: Time) -> MarkerRmv:
        return MarkerRmv(message, time)


class MarkerArrayMessage(BaseMessage):
    """Class to process a MarkerArray message.
    Convert a message of type MarkerArray into a list of MarkerRmv.
    Args:
        message_type (type): The type of message to process (MarkerArray).
    """

    def __init__(self):
        super().__init__(MarkerArray)

    @staticmethod
    def process(message: MarkerArray, time: Time) -> list[MarkerRmv]:
        return [MarkerRmv(marker, time) for marker in message.markers]

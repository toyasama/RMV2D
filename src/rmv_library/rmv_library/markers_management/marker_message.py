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

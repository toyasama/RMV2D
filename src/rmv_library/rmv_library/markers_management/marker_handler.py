from .markers import MarkerRmv

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from builtin_interfaces.msg import Time
from .marker_message import MarkerArrayMessage, MarkerMessage
from threading import RLock, Thread
import time


class MarkersHandler:
    """
    Class to handle the markers list and manage the addition and deletion of markers.
    It uses threads to process incoming messages and delete expired markers.
    New markers are added to a temporary list and then merged into the main markers list every 30ms.
    Expired markers are deleted from the main markers list every 500ms.
    The class is designed to be thread-safe, using locks to manage access to shared resources.
    """

    def __init__(self):
        self.__markers: dict[tuple[str, int], MarkerRmv] = {}
        self.__new_markers: dict[tuple[str, int], MarkerRmv] = {}
        self.__new_msgs: list[MarkerArray | Marker] = []
        self.__lock_markers_list = RLock()
        self.__lock_new_msgs = RLock()
        self.__running = True
        self.__delete_markers_thread: Thread = Thread(
            target=self.__deleteExpiredMarkers
        )
        self.__delete_markers_thread.start()
        self.__merge_markers_thread: Thread = Thread(target=self.__mergeNewMarkers)
        self.__merge_markers_thread.start()

    def stop(self):
        if not self.__running:
            return
        print("Terminating markers handler threads...")
        self.__running = False
        self.__delete_markers_thread.join()
        self.__merge_markers_thread.join()
        print("Markers handler threads terminated.")

    @property
    def markers(self) -> list[MarkerRmv]:
        return list(self.__markers.values())

    def addMarker(self, marker: Marker | MarkerArray):
        """The marker is added to the list of new messages, which will be processed in the next cycle.
        Args:
            marker (Marker | MarkerArray): The marker to be added.
        """
        with self.__lock_new_msgs:
            self.__new_msgs.append(marker)

    def clearMarkersList(self):
        with self.__lock_markers_list:
            self.__markers.clear()

    def __mergeNewMarkers(self):
        while self.__running:
            self.__processMessage()
            with self.__lock_markers_list:
                self.__markers.update(self.__new_markers)
            self.__new_markers.clear()
            time.sleep(0.03)

    def __timeAsMsg(self) -> Time:
        """Get the current time as a Time message."""
        now = time.time()
        seconds = int(now)
        nanoseconds = int((now - seconds) * 1e9)
        return Time(sec=seconds, nanosec=nanoseconds)

    def __insertNewMarkers(self, marker: MarkerRmv, reception_time: Time):
        if not marker.isExpired(reception_time):
            self.__new_markers[marker.identifier] = marker

    def __processMessage(self):
        with self.__lock_new_msgs:
            new_msgs = self.__new_msgs.copy()
            self.__new_msgs.clear()
        reception_time = self.__timeAsMsg()
        for msg in new_msgs:
            if isinstance(msg, Marker):
                marker = MarkerMessage.process(msg, reception_time)
                self.__insertNewMarkers(marker, reception_time)

            elif isinstance(msg, MarkerArray):
                marker_list: list[MarkerRmv] = MarkerArrayMessage.process(
                    msg, reception_time
                )
                for marker in marker_list:
                    self.__insertNewMarkers(marker, reception_time)

    def __deleteExpiredMarkers(self):
        while self.__running:
            current_time = self.__timeAsMsg()
            with self.__lock_markers_list:
                expired_keys = [
                    identifier
                    for identifier, marker in self.__markers.items()
                    if marker.isExpired(current_time)
                ]
                for key in expired_keys:
                    del self.__markers[key]
            time.sleep(0.5)

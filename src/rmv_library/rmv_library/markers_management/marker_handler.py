from .markers import MarkerRmv

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node

from .marker_message import MarkerArrayMessage, MarkerMessage
from threading import RLock, Thread
import time


class MarkersHandler:
    def __init__(self, node: Node):
        self.__node = node
        self.__markers: dict[tuple[str, int], MarkerRmv] = {}
        self.__new_markers: dict[tuple[str, int], MarkerRmv] = {}
        self.__new_msgs: list[MarkerArray | Marker] = []
        self.__lock_markers_list = RLock()
        self.__lock_new_msgs = RLock()
        self.__running = True
        self.__delete_markers_thread: Thread = Thread(target=self._deleteExpiredMarkers)
        self.__delete_markers_thread.start()
        self.__merge_markers_thread: Thread = Thread(target=self.__mergeNewMarkers)
        self.__merge_markers_thread.start()

    def __mergeNewMarkers(self):
        while self.__running:
            self.__processMessage()
            with self.__lock_markers_list:
                self.__markers.update(self.__new_markers)
            self.__new_markers.clear()
            time.sleep(0.03)

    def __del__(self):
        print("Terminating markers handler threads...")
        self.__running = False
        self.__delete_markers_thread.join()
        self.__merge_markers_thread.join()
        print("Markers handler threads terminated.")

    def addMarker(self, marker: Marker | MarkerArray):
        """Add a new marker to the markers list.

        Args:
            marker (Marker | MarkerArray): The marker to be added.

        """
        with self.__lock_new_msgs:
            self.__new_msgs.append(marker)

    def __processMessage(self):
        with self.__lock_new_msgs:
            new_msgs = self.__new_msgs.copy()
            self.__new_msgs.clear()
        reception_time = self.__node.get_clock().now().to_msg()
        for msg in new_msgs:
            if isinstance(msg, Marker):
                marker = MarkerMessage.process(msg, reception_time)
                if marker.isExpired(reception_time):
                    print(f"Marker {marker.identifier} expired before adding")
                    continue
                self.__new_markers[marker.identifier] = marker

            elif isinstance(msg, MarkerArray):
                marker_list: list[MarkerRmv] = MarkerArrayMessage.process(
                    msg, reception_time
                )
                for marker in marker_list:
                    if marker.isExpired(reception_time):
                        print(f"Marker {marker.identifier} expired before adding")
                        continue
                    self.__new_markers[marker.identifier] = marker

    @property
    def markers(self) -> list[MarkerRmv]:
        return list(self.__markers.values())

    def clearMarkersList(self):
        """Clear the markers list."""
        with self.__lock_markers_list:
            self.__markers.clear()

    def _deleteExpiredMarkers(self):
        """Delete the expired markers from the markers list."""
        while self.__running:
            current_time = self.__node.get_clock().now().to_msg()
            with self.__lock_markers_list:
                expired_keys = [
                    identifier
                    for identifier, marker in self.__markers.items()
                    if marker.isExpired(current_time)
                ]
                for key in expired_keys:
                    del self.__markers[key]
            time.sleep(0.5)

#!/usr/bin/env python3

"""
Waypoint editor node
author: Toshiki Nakamura
"""

import copy

import rospy
import yaml
from std_srvs.srv import Trigger, TriggerResponse

from waypoint_editor_ros.srv import EditPoint, EditPointResponse


class WaypointEditor:
    """class for waypoint editor node

    Attributes:
        _waypoint_file (str): waypoint file path
        _edit_waypoint_server (rospy.Service): edit waypoint server
        _undo_waypoint_server (rospy.Service): undo waypoint server
        _redo_waypoint_server (rospy.Service): redo waypoint server
        _version (int): version number
        _waypoints_backup (list): backup of waypoints
        _waypoints (list): waypoints
    """

    def __init__(self) -> None:
        """Initialize waypoint editor node

        Args:
            None

        Returns:
            None
        """

        rospy.init_node("waypoint_editor")
        self._waypoint_file: str = rospy.get_param(
            "~waypoint_file", "waypoints.yaml"
        )
        self._edit_waypoint_server: rospy.Service = rospy.Service(
            "~edit_waypoint", EditPoint, self._handle_edit_waypoint
        )
        self._undo_waypoint_server: rospy.Service = rospy.Service(
            "~undo_waypoint", Trigger, self._handle_undo_waypoint
        )
        self._redo_waypoint_server: rospy.Service = rospy.Service(
            "~redo_waypoint", Trigger, self._handle_redo_waypoint
        )

        # Print parameters
        rospy.loginfo(f"{rospy.get_name()} node has started...")
        rospy.loginfo("")
        rospy.loginfo("Parameters:")
        rospy.loginfo(f"  waypoint_file: {self._waypoint_file}")
        rospy.loginfo("")

        # waypoints
        self._version: int = -1
        self._waypoints_backup: list = list()
        self._waypoints: list = self._load(self._waypoint_file)
        if len(self._waypoints) > 0:
            self._backup(self._waypoints)
            self._save(self._waypoints, self._waypoint_file)

    def _handle_edit_waypoint(self, req: EditPoint) -> EditPointResponse:
        """Handle edit waypoint service

        Args:
            req (EditPoint): request

        Returns:
            EditPointResponse: response
        """

        res: EditPointResponse = EditPointResponse()
        if req.mode != "add" and (
            req.id < 0 or len(self._waypoints) <= req.id
        ):
            res.success = False
            res.message = "The id is out of range"
            return res
        res.success = True
        res.message = (
            self._edit(self._waypoints, req)
            + f" (ver. {self._version}/{len(self._waypoints_backup) - 1})"
        )
        self._save(self._waypoints, self._waypoint_file)
        return res

    def _handle_undo_waypoint(self, req: Trigger) -> TriggerResponse:
        """Handle undo waypoint service

        Args:
            req (Trigger): request

        Returns:
            TriggerResponse: response
        """

        print(req)
        res: TriggerResponse = TriggerResponse()
        res.success = self._undo()
        if res.success:
            res.message = f"Undo to version \
            {self._version}/{len(self._waypoints_backup) - 1}"
        else:
            res.message = "[ERROR] Older version does not exist"
        return res

    def _handle_redo_waypoint(self, req: Trigger) -> TriggerResponse:
        """Handle redo waypoint service

        Args:
            req (Trigger): request

        Returns:
            TriggerResponse: response
        """

        print(req)
        res: TriggerResponse = TriggerResponse()
        res.success = self._redo()
        if res.success:
            res.message = f"Redo to version \
                {self._version}/{len(self._waypoints_backup) - 1}"
        else:
            res.message = "[ERROR] Newer version does not exist"
        return res

    def _load(self, file_path: str) -> list:
        """Load waypoints from file

        Args:
            file_path (str): file path

        Returns:
            list: waypoints
        """

        try:
            with open(file_path, "r", encoding="utf-8") as file:
                waypoint: list = yaml.safe_load(file)
                rospy.loginfo(f"Load waypoints from {file_path}")
                return waypoint
        except FileNotFoundError:
            rospy.logwarn(f"Create new file: {file_path}")
            return list()

    def _backup(self, waypoints: list) -> None:
        """Backup waypoints

        Args:
            waypoints (list): waypoints

        Returns:
            None
        """

        if len(self._waypoints_backup) - 1 > self._version:
            self._waypoints_backup = copy.deepcopy(
                self._waypoints_backup[: self._version + 1]
            )
        self._modify_ids(waypoints)
        self._waypoints_backup.append(copy.deepcopy(waypoints))
        self._version += 1

    def _undo(self) -> bool:
        """Undo waypoints

        Returns:
            bool: True if undo is successful, False otherwise
        """

        if self._version > 0:
            self._version -= 1
            self._waypoints = copy.deepcopy(
                self._waypoints_backup[self._version]
            )
            self._save(self._waypoints, self._waypoint_file)
            return True
        else:
            return False

    def _redo(self) -> bool:
        """Redo waypoints

        Returns:
            bool: True if redo is successful, False otherwise
        """

        if self._version < len(self._waypoints_backup) - 1:
            self._version += 1
            self._waypoints = copy.deepcopy(
                self._waypoints_backup[self._version]
            )
            self._save(self._waypoints, self._waypoint_file)
            return True
        else:
            return False

    def _edit(self, waypoints: list, edit_point: EditPoint) -> str:
        """Edit waypoints

        Args:
            waypoints (list): waypoints
            edit_point (EditPoint): edit point

        Returns:
            str: result message
        """

        if edit_point.mode == "add":
            waypoints.append(
                {
                    "id": len(waypoints),
                    "x": edit_point.x,
                    "y": edit_point.y,
                    "yaw": edit_point.yaw,
                }
            )
        elif edit_point.mode == "modify":
            for waypoint in waypoints:
                if waypoint["id"] == edit_point.id:
                    waypoint["x"] = edit_point.x
                    waypoint["y"] = edit_point.y
                    waypoint["yaw"] = edit_point.yaw
                    break
        elif edit_point.mode == "insert":
            for waypoint in waypoints:
                if waypoint["id"] >= edit_point.id:
                    waypoint["id"] += 1
            waypoints.insert(
                edit_point.id,
                {
                    "id": edit_point.id,
                    "x": edit_point.x,
                    "y": edit_point.y,
                    "yaw": edit_point.yaw,
                },
            )
        elif edit_point.mode == "delete":
            for i, waypoint in enumerate(waypoints):
                if waypoint["id"] == edit_point.id:
                    waypoints.pop(i)
                    break

        self._backup(waypoints)
        return edit_point.mode.capitalize() + " waypoint"

    def _save(self, waypoints: list, file_path: str) -> None:
        """Save waypoints to file

        Args:
            waypoints (list): waypoints
            file_path (str): file path

        Returns:
            None
        """

        self._modify_ids(waypoints)
        with open(file_path, "w", encoding="utf-8") as file:
            yaml.dump(waypoints, file)
        rospy.loginfo(f"Save waypoints to {file_path}")

    def _modify_ids(self, waypoints: list) -> None:
        """Modify waypoint ids

        Args:
            waypoints (list): waypoints

        Returns:
            None
        """

        for i, waypoint in enumerate(waypoints):
            waypoint["id"] = i


if __name__ == "__main__":
    try:
        node = WaypointEditor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception caught")

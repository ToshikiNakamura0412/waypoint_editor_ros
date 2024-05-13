#!/usr/bin/env python3

import copy

import rospy
import yaml
from std_srvs.srv import Trigger, TriggerResponse

from waypoint_editor_ros.srv import EditPoint, EditPointResponse


class WaypointEditor:
    def __init__(self) -> None:
        rospy.init_node("waypoint_editor")
        self._waypoint_file = rospy.get_param("~waypoint_file", "waypoints.yaml")
        self._edit_waypoint_server = rospy.Service(
            "~edit_waypoint", EditPoint, self._handle_edit_waypoint
        )
        self._undo_waypoint_server = rospy.Service(
            "~undo_waypoint", Trigger, self._handle_undo_waypoint
        )
        self._redo_waypoint_server = rospy.Service(
            "~redo_waypoint", Trigger, self._handle_redo_waypoint
        )

        # Print parameters
        rospy.loginfo(f"{rospy.get_name()} node has started...")
        rospy.loginfo("")
        rospy.loginfo("Parameters:")
        rospy.loginfo(f"  waypoint_file: {self._waypoint_file}")
        rospy.loginfo("")

        # waypoints
        self._version = -1
        self._waypoints_backup = list()
        self._waypoints = self._load(self._waypoint_file)
        self._backup(self._waypoints)
        self._save(self._waypoints, self._waypoint_file)

    def _handle_edit_waypoint(self, req: EditPoint) -> EditPointResponse:
        res = EditPointResponse()
        if req.mode != "add" and (req.id < 0 or len(self._waypoints) <= req.id):
            res.success = False
            res.message = "The id is out of range"
            return res
        res.success = True
        res.message = self._edit(self._waypoints, req) + f" (ver. {self._version}/{len(self._waypoints_backup) - 1})"
        self._save(self._waypoints, self._waypoint_file)
        return res

    def _handle_undo_waypoint(self, req: Trigger) -> TriggerResponse:
        res = TriggerResponse()
        res.success = self._undo()
        if res.success:
            res.message = (
                f"Undo to version {self._version}/{len(self._waypoints_backup) - 1}"
            )
        else:
            res.message = "[ERROR] Older version does not exist"
        return res

    def _handle_redo_waypoint(self, req: Trigger) -> TriggerResponse:
        res = TriggerResponse()
        res.success = self._redo()
        if res.success:
            res.message = (
                f"Redo to version {self._version}/{len(self._waypoints_backup) - 1}"
            )
        else:
            res.message = "[ERROR] Newer version does not exist"
        return res

    def _load(self, file_path: str) -> list:
        try:
            with open(file_path, "r") as file:
                waypoint = yaml.safe_load(file)
                rospy.loginfo(f"Load waypoints from {file_path}")
                return waypoint
        except FileNotFoundError:
            rospy.logwarn(f"Create new file: {file_path}")
            return list()

    def _backup(self, waypoints: list) -> None:
        if len(self._waypoints_backup) - 1 > self._version:
            self._waypoints_backup = copy.deepcopy(
                self._waypoints_backup[: self._version + 1]
            )
        self._modify_ids(waypoints)
        self._waypoints_backup.append(copy.deepcopy(waypoints))
        self._version += 1

    def _undo(self) -> bool:
        if self._version > 0:
            self._version -= 1
            self._waypoints = copy.deepcopy(self._waypoints_backup[self._version])
            self._save(self._waypoints, self._waypoint_file)
            return True
        else:
            return False

    def _redo(self) -> bool:
        if self._version < len(self._waypoints_backup) - 1:
            self._version += 1
            self._waypoints = copy.deepcopy(self._waypoints_backup[self._version])
            self._save(self._waypoints, self._waypoint_file)
            return True
        else:
            return False

    def _edit(self, waypoints: list, edit_point: EditPoint) -> str:
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
            for i in range(len(waypoints)):
                if waypoints[i]["id"] == edit_point.id:
                    waypoints.pop(i)
                    break

        self._backup(waypoints)
        return edit_point.mode.capitalize() + " waypoint"

    def _save(self, waypoints: list, file_path: str) -> None:
        self._modify_ids(waypoints)
        with open(file_path, "w") as file:
            yaml.dump(waypoints, file)
        rospy.loginfo(f"Save waypoints to {file_path}")

    def _modify_ids(self, waypoints: list) -> None:
        for i, waypoint in enumerate(waypoints):
            waypoint["id"] = i


if __name__ == "__main__":
    try:
        node = WaypointEditor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception caught")
        pass

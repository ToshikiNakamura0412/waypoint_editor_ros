#!/usr/bin/env python3

"""
Edit client node
author: Toshiki Nakamura
"""

import signal
import sys

import rospy
from edit_client_ui import Ui_Dialog
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QDialog
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion

from waypoint_editor_ros.srv import EditPoint, EditPointResponse


class EditClient(QDialog):
    """class for edit client

    Attributes:
        _edit_client (rospy.ServiceProxy): edit waypoint service client
        _goal_pose_sub (rospy.Subscriber): goal pose subscriber
        _redo_client (rospy.ServiceProxy): redo waypoint service client
        _undo_client (rospy.ServiceProxy): undo waypoint service client
        _edit_point (EditPoint): edit point
    """

    def __init__(self, parent=None):
        """Initialize edit client

        Args:
            parent (QWidget): parent widget

        Returns:
            None
        """

        super(EditClient, self).__init__(parent=parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)

        rospy.init_node("edit_client")
        rospy.wait_for_service("/waypoint_editor/edit_waypoint")
        self._edit_client: rospy.ServiceProxy = rospy.ServiceProxy(
            "/waypoint_editor/edit_waypoint", EditPoint
        )
        self._undo_client: rospy.ServiceProxy = rospy.ServiceProxy(
            "/waypoint_editor/undo_waypoint", Trigger
        )
        self._redo_client: rospy.ServiceProxy = rospy.ServiceProxy(
            "/waypoint_editor/redo_waypoint", Trigger
        )
        self._goal_pose_sub: rospy.Subscriber = rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self._goal_pose_callback,
            queue_size=1,
        )
        self._edit_point: EditPoint = EditPoint()

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        """Goal pose callback

        Args:
            msg (PoseStamped): goal pose message

        Returns:
            None
        """

        self._edit_point.x = msg.pose.position.x
        self._edit_point.y = msg.pose.position.y
        _, _, self._edit_point.yaw = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        self._set_text(self._edit_point)

    def _set_text(self, edit_point: EditPoint) -> None:
        """Set text

        Args:
            edit_point (EditPoint): edit point

        Returns:
            None
        """

        self.ui.value_1_x.setText(str(edit_point.x))
        self.ui.value_2_y.setText(str(edit_point.y))
        self.ui.value_3_yaw.setText(str(edit_point.yaw))

    def clicked_add(self) -> None:
        """Clicked add

        Returns:
            None
        """

        self._edit_point.mode = "add"

    def clicked_modify(self) -> None:
        """Clicked modify

        Returns:
            None
        """

        self._edit_point.mode = "modify"

    def clicked_insert(self) -> None:
        """Clicked insert

        Returns:
            None
        """

        self._edit_point.mode = "insert"

    def clicked_delete(self) -> None:
        """Clicked delete

        Returns:
            None
        """

        self._edit_point.mode = "delete"

    def undo(self) -> None:
        """Call undo service

        Returns:
            None
        """

        res: TriggerResponse = self._undo_client()
        self.ui.text_result.setText(res.message)

    def redo(self) -> None:
        """Call redo service

        Returns:
            None
        """
        res: TriggerResponse = self._redo_client()
        self.ui.text_result.setText(res.message)

    def call(self) -> None:
        """Call edit waypoint service

        Returns:
            None
        """

        self.ui.text_result.setText("")
        try:
            self._edit_point.id = int(self.ui.edit_id.text())
        except ValueError:
            pass
        if not hasattr(self._edit_point, "mode"):
            self.ui.text_result.setText("[ERROR] Please set mode")
            return
        if (
            not hasattr(self._edit_point, "x")
            or not hasattr(self._edit_point, "y")
            or not hasattr(self._edit_point, "yaw")
        ) and self._edit_point.mode != "delete":
            self.ui.text_result.setText("[ERROR] Please set goal")
            return
        if self._edit_point.mode != "add" and not hasattr(
            self._edit_point, "id"
        ):
            self.ui.text_result.setText("[ERROR] Please set id")
            return
        if self._edit_point.mode != "add" and (
            self._edit_point.id < 0 or not isinstance(self._edit_point.id, int)
        ):
            self.ui.text_result.setText(
                "[ERROR] Invalid id. Please set a natural number."
            )
            return

        res: EditPointResponse = self._edit_client(
            id=self._edit_point.id if hasattr(self._edit_point, "id") else -1,
            x=self._edit_point.x if hasattr(self._edit_point, "x") else 0.0,
            y=self._edit_point.y if hasattr(self._edit_point, "y") else 0.0,
            yaw=(
                self._edit_point.yaw
                if hasattr(self._edit_point, "yaw")
                else 0.0
            ),
            mode=self._edit_point.mode,
        )
        self.ui.text_result.setText(res.message)

        tmp_mode: str = self._edit_point.mode
        self._edit_point = EditPoint()
        self._edit_point.mode = tmp_mode
        self._clear_text()

    def _clear_text(self) -> None:
        """Clear text

        Returns:
            None
        """

        self.ui.value_1_x.setText("")
        self.ui.value_2_y.setText("")
        self.ui.value_3_yaw.setText("")


if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        window = EditClient()
        window.show()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception caught")

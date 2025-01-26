#!/usr/bin/env python3
from collections import namedtuple
import rospy

# Libraries for reading screen coordinates of the mouse and of the TurtleSim window.
import pynput
from ewmh import EWMH

from turtlesim.msg import Pose

SIM_MAX_CORNER = 11.1
SIM_MIN_CORNER = 0.0

class TurtleMouseNode:
    # Helper namedtuple type which stores `x` and `y`, so we can access them by name instead of [0] and [1].
    ScreenCoordinate = namedtuple("ScreenCoordinate", ["x", "y"])

    def __init__(self):
        # For reading the mouse position, we use pynput.mouse.
        self.mouse = pynput.mouse.Controller()
        # For reading the TurtleSim window position, we use the ewmh library (`extended window manager hints`).
        self.window_manager = EWMH()

        # Wraps `self.recurse_windows` in a lambda which takes in the unused TimerEvent argument (_), so we can
        # register it as a callback for a `rospy.Timer`.
        update_turtlesim_window_position = lambda _: self.recurse_windows(self.window_manager.root)

        # At this point, we have no idea about the TurtleSim window position, so set it to None to indicate this.
        self.turtlesim_upperleft_xy = None
        self.turtlesim_size = None
        # Call the update function once now, in order to fully initialize the TurtleSim window position and size.
        update_turtlesim_window_position(None)

        # Call the TurtleSim window update at 1 Hz -- not too often, because it is somewhat expensive.
        # Note: keep in mind that `rospy.Duration` takes in the period as an argument, and not the frequency.
        rospy.Timer(rospy.Duration(1.0 / 1.0), update_turtlesim_window_position)

        # turtlemouse_pose publisher 
        self.turtlemouse_pose = rospy.Publisher('turtlemouse_pose', Pose, queue_size=1)

        # TODO: add a timer which calls `self.publish_mouse_position` at a 30 Hz frequency.
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_mouse_position)

    # Finds the TurtleSim window and stores its position and size in `self.turtlesim_window_xy` and `self.turtlesim_window_size`.
    # You can skip reading the details of how `recurse_windows` works; inner workings of this function will not be covered by the exam.
    # BEGIN code which you can skip studying
    def recurse_windows(self, window):
        try:
            # Recursion base case: found the TurtleSim window.
            if window.get_wm_name() == "TurtleSim":
                my_geometry = window.get_geometry()
                parent = window.query_tree().parent
                parent_geometry = parent.get_geometry()
                parent_parent = parent.query_tree().parent
                parent_parent_geometry = parent_parent.get_geometry()
                # GNOME and Unity window managers are reparenting the windows like this when they are making space for the window menu
                # and title bar. Therefore, we need to add these offsets in order to get the position of the TurtleSim window itself.
                self.turtlesim_upperleft_xy = TurtleMouseNode.ScreenCoordinate(
                    parent_parent_geometry.x + parent_geometry.x + my_geometry.x,
                    parent_parent_geometry.y + parent_geometry.y + my_geometry.y,
                )
                # The TurtleSim window size should be 500x500, if everything is correct.
                self.turtlesim_size = TurtleMouseNode.ScreenCoordinate(
                    my_geometry.width, my_geometry.height
                )
                return
        except:
            # get_wm_name() seems to fail for some evil windows. Ignore.
            pass

        # This is not the TurtleSim window. Recurse further to find it.
        for child in window.query_tree().children:
            self.recurse_windows(child)
    ### END code which you can skip studying


    # This is the callback function which should get called periodically to handle mouse position updates.
    def publish_mouse_position(self, _):
        # We have no idea (yet) where TurtleSim is. We have nothing to do, so abort.
        if self.turtlesim_upperleft_xy is None:
            return

        # Copy the mouse position into the `ScreenCoordinate` namedtuple type.
        mouse_xy = TurtleMouseNode.ScreenCoordinate(*self.mouse.position)

        # Debug print to help you figure out the coordinates. You can remove/comment it out once it is no longer needed.
        """
        print(f"mouse_xy: {mouse_xy.x} {mouse_xy.y}, " +
              f"turtlesim_upperleft_xy: {self.turtlesim_upperleft_xy.x}, {self.turtlesim_upperleft_xy.y}, " +
              f"turtlesim_window_size: {self.turtlesim_size.x}, {self.turtlesim_size.y}")
        """

        # Variables that will contain the transformed position of the mouse
        
        # This division by self.turtlesim_size.x * 11.1 is to convert from the screen 
        # coords to TurtleSim coords. The first part unifies the screen 
        # coords and TurtleSim coords under the same reference point.
        self.new_mouse_pose_x = (
            (mouse_xy.x - self.turtlesim_upperleft_xy.x) / self.turtlesim_size.x * SIM_MAX_CORNER
        )

        self.new_mouse_pose_y = (
            (self.turtlesim_size.y - (mouse_xy.y - self.turtlesim_upperleft_xy.y)) / self.turtlesim_size.y * SIM_MAX_CORNER
        )

        # Clamping the result to (0.0 - 11.1)
        self.new_mouse_pose_x = clamp(SIM_MIN_CORNER, self.new_mouse_pose_x, SIM_MAX_CORNER)
        self.new_mouse_pose_y = clamp(SIM_MIN_CORNER, self.new_mouse_pose_y, SIM_MAX_CORNER)

        # Publishing the turtlemouse_pose calculated before.
        self.pose = Pose()
        self.pose.x = self.new_mouse_pose_x
        self.pose.y = self.new_mouse_pose_y

        self.turtlemouse_pose.publish(self.pose)

def clamp(minvalue, value, maxvalue):
        return max(minvalue, min(value, maxvalue))

if __name__ == "__main__":
    rospy.init_node("turtle_mouse")
    node = TurtleMouseNode()
    rospy.spin()

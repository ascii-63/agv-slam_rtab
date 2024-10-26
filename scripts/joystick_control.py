#!/usr/bin/env python3
# encoding: utf-8

import rospy
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Mapping of joystick axes and buttons
AXES_MAP = ['lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y']
BUTTON_MAP = [
    'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2',
    'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''
]


def value_mapping(x, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control')
        self.max_linear = rospy.get_param('~max_linear', 0.4)
        self.max_angular = rospy.get_param('~max_angular', 2.0)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel', '/cmd_vel')

        # Set up subscribers and publishers
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1)

        # Initialize state variables
        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))

    def axes_callback(self, axes):
        """
        Callback function to handle joystick axes movement.
        """
        twist = Twist()
        twist.linear.y = value_mapping(
            axes['lx'], -1, 1, -self.max_linear, self.max_linear)
        twist.linear.x = value_mapping(
            axes['ly'], -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = value_mapping(
            axes['rx'], -1, 1, -self.max_angular, self.max_angular)
        self.cmd_vel_pub.publish(twist)

    def start_callback(self, state):
        """
        Callback for the 'start' button.
        """
        if state == ButtonState.Pressed:
            rospy.loginfo("Start button pressed. Stopping robot.")
            # Publish zero velocities to stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

    def select_callback(self, state):
        """
        Callback for the 'select' button.
        """
        if state == ButtonState.Pressed:
            rospy.loginfo("Select button pressed.")
            # Perform any reset or special action here

    def joy_callback(self, joy_msg):
        """
        Callback function to handle joystick messages.
        """
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False

        # Process hat switches
        hat_x = axes.get('hat_x', 0.0)
        hat_y = axes.get('hat_y', 0.0)
        hat_xl = int(hat_x < -0.5)
        hat_xr = int(hat_x > 0.5)
        hat_yu = int(hat_y > 0.5)
        hat_yd = int(hat_y < -0.5)

        # Extend buttons with hat switch directions
        buttons = list(joy_msg.buttons)
        # The last 0 is a placeholder
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))

        # Check for changes in axes
        for key in axes:
            if self.last_axes.get(key, 0.0) != axes[key]:
                axes_changed = True
                break

        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(f"Error in axes_callback: {e}")

        # Check for button state changes
        for key in buttons:
            value = buttons[key]
            last_value = self.last_buttons.get(key, 0)
            new_state = ButtonState.Normal
            if value != last_value:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            elif value > 0:
                new_state = ButtonState.Holding

            callback_name = f"{key}_callback"
            if new_state != ButtonState.Normal and hasattr(self, callback_name):
                rospy.loginfo(f"{key}: {new_state}")
                try:
                    getattr(self, callback_name)(new_state)
                except Exception as e:
                    rospy.logerr(f"Error in {callback_name}: {e}")

        self.last_buttons = buttons
        self.last_axes = axes


if __name__ == "__main__":
    try:
        controller = JoystickController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

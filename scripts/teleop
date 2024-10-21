#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Control Your Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.05),  # Slightly reduce angular sensitivity
    'c': (1, .95),   # Slightly reduce angular sensitivity
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    prev_turn = 0  # Initialize previous turn for filtering
    alpha = 0.5    # Low-pass filter coefficient for smoothing angular control

    try:
        print(msg)
        print(vels(speed, turn))
        while (1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            # Dead zone for angular movement to prevent small drift
            if abs(target_turn) < 0.05:
                target_turn = 0

            # Smooth speed adjustments
            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.01)  # Finer speed adjustment
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.01)

            # Smooth turn adjustments with low-pass filtering
            control_turn = alpha * target_turn + (1 - alpha) * prev_turn  # Apply smoothing filter
            prev_turn = control_turn  # Update previous turn value for the next iteration

            # Publish Twist message
            twist = Twist()
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
